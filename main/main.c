/*
 * main.c – RDS-Decoder für den Original-ESP32 (ESP-IDF 6.x)
 *
 * Hardware:
 *   MPX-Signal (nach analoger Eingangsstufe: AC-Kopplung, Bias auf halben ADC-Eingangsbereich,
 *   Bandpass um 57 kHz mit Verstärkung, Tiefpass gegen Aliasing) an GPIO34
 *   (= ADC1_CH6, nur Eingang). WLAN/Bluetooth werden nicht initialisiert.
 *
 * Software-Aufbau:
 *   dsp_task      (Core 1, hohe Prio)  ADC-DMA lesen → rds_dsp → rds_decoder
 *   console_task  (Core 0, niedrige Prio) Rohgruppen + alle N Sekunden Zusammenfassung
 *
 * Der ADC läuft im "continuous mode" (DMA). Die tatsächliche Abtastrate wird
 * beim Start über die Systemuhr gemessen (siehe measure_rate()), damit die
 * Costas-Loop mit der richtigen Restträgerfrequenz startet.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_continuous.h"

#include "rds_dsp.h"
#include "rds_group.h"

/* ============================ Konfiguration ============================ */
#define ADC_UNIT_USED        ADC_UNIT_1
#define ADC_CHANNEL_USED     ADC_CHANNEL_6        /* GPIO34 auf dem ESP32            */
//#define ADC_ATTEN_USED       ADC_ATTEN_DB_12      /* ~0 … 3,1 V Eingangsbereich      */
#define ADC_ATTEN_USED       ADC_ATTEN_DB_0       /* 100 mV ~ 950 mV                 */
// #define ADC_ATTEN_USED       ADC_ATTEN_DB_2_5     /* 100 mV ~ 1250 mV                */
// #define ADC_ATTEN_USED       ADC_ATTEN_DB_6       /* 150 mV ~ 1750 mV                */
// #define ADC_ATTEN_USED       ADC_ATTEN_DB_11      /* 150 mV ~ 2450 mV                */

#define ADC_SAMPLE_RATE_HZ   228000               /* 4 × 57 kHz                      */

#define ADC_FRAME_BYTES      2048                 /* ein DMA-Frame ≈ 4,5 ms          */
#define ADC_POOL_BYTES       (ADC_FRAME_BYTES * 8)/* Ringpuffer im Treiber           */
#define ADC_MAX_SAMPLES      (ADC_FRAME_BYTES / SOC_ADC_DIGI_RESULT_BYTES)

#define MEASURE_SECONDS      8                    /* Dauer der Abtastraten-Messung   */
#define RETUNE_THRESHOLD_HZ  100.0f               /* ab dieser Trägerabweichung neu einstellen */

#define PRINT_INTERVAL_MS    5000                 /* Zusammenfassung alle 5 s        */
#define SHOW_RAW_GROUPS      1                    /* 1 = jede Gruppe/Block anzeigen  */

static const char *TAG = "rds";

/* ============================== Globale Zustände ============================== */
static adc_continuous_handle_t s_adc;
static uint8_t  s_raw[ADC_FRAME_BYTES];           /* Rohdaten vom DMA                */
static uint16_t s_smp[ADC_MAX_SAMPLES];           /* extrahierte 12-Bit-Werte        */
static volatile uint32_t s_ovf_count;             /* Pool-Überläufe (Samples verloren) */
static uint32_t s_req_rate_hz = ADC_SAMPLE_RATE_HZ;
static float    s_fs_actual   = ADC_SAMPLE_RATE_HZ;

static rds_dsp_t     s_dsp;
static rds_decoder_t s_dec;
static SemaphoreHandle_t s_mtx;                   /* schützt s_dec und s_dsp-Status  */
static QueueHandle_t     s_grp_q;                 /* Rohgruppen für die Konsole      */
static uint32_t          s_now_ms;                /* Zeitstempel des aktuellen Frames */

/* ============================== ADC ============================== */

/* ISR-Callback: DMA-Pool voll → wir sind nicht schnell genug gewesen. */
static bool IRAM_ATTR on_pool_ovf(adc_continuous_handle_t h,
                                  const adc_continuous_evt_data_t *e, void *u)
{
    s_ovf_count++;
    return false;
}

static esp_err_t adc_begin(uint32_t rate_hz)
{
    adc_continuous_handle_cfg_t hcfg = {
        .max_store_buf_size = ADC_POOL_BYTES,
        .conv_frame_size    = ADC_FRAME_BYTES,
    };
    ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&hcfg, &s_adc), TAG, "new_handle");

    adc_digi_pattern_config_t pat = {
        .atten     = ADC_ATTEN_USED,
        .channel   = ADC_CHANNEL_USED,
        .unit      = ADC_UNIT_USED,
        .bit_width = ADC_BITWIDTH_12,
    };
    adc_continuous_config_t cfg = {
        .pattern_num    = 1,
        .adc_pattern    = &pat,
        .sample_freq_hz = rate_hz,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,     /* einziges Format beim ESP32 */
    };
    ESP_RETURN_ON_ERROR(adc_continuous_config(s_adc, &cfg), TAG, "config");

    adc_continuous_evt_cbs_t cbs = { .on_pool_ovf = on_pool_ovf };
    ESP_RETURN_ON_ERROR(adc_continuous_register_event_callbacks(s_adc, &cbs, NULL), TAG, "cbs");
    return adc_continuous_start(s_adc);
}

static void adc_end(void)
{
    adc_continuous_stop(s_adc);
    adc_continuous_deinit(s_adc);
    s_adc = NULL;
}

/* Liest einen DMA-Frame und zieht die 12-Bit-Werte heraus. Rückgabe: Anzahl Samples, <0 bei Fehler. */
static int adc_read_samples(uint32_t timeout_ms)
{
    uint32_t len = 0;
    esp_err_t r = adc_continuous_read(s_adc, s_raw, sizeof(s_raw), &len, timeout_ms);
    if (r != ESP_OK) return -1;

    int n = 0;
    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= len; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&s_raw[i];
        if (p->type1.channel != ADC_CHANNEL_USED) continue;      /* Sicherheitsprüfung */
        s_smp[n++] = (uint16_t)p->type1.data;
    }
    return n;
}

/* Misst die echte Abtastrate: Samples zwischen erstem und letztem Frame / Zeitdifferenz.
 * Läuft ohne Last (kein DSP), damit die Frames pünktlich abgeholt werden. */
static float measure_rate(void)
{
    int64_t t_end = esp_timer_get_time() + (int64_t)MEASURE_SECONDS * 1000000;
    int64_t t_first = 0, t_last = 0;
    uint64_t cnt = 0;
    bool first = true;

    while (esp_timer_get_time() < t_end) {
        int n = adc_read_samples(200);
        int64_t now = esp_timer_get_time();
        if (n <= 0) continue;
        if (first) { first = false; t_first = now; }     /* erster Frame = Zeitanker  */
        else       { cnt += (uint64_t)n; t_last = now; }
    }
    if (t_last <= t_first) return (float)s_req_rate_hz;
    return (float)((double)cnt * 1.0e6 / (double)(t_last - t_first));
}

/* ============================== DSP-Task ============================== */

/* Bit aus dem Demodulator → Gruppendekoder (Mutex ist in dsp_task bereits gehalten). */
static void on_bit(uint8_t bit, void *ctx)
{
    rds_decoder_push_bit(&s_dec, bit, s_now_ms);
}

/* Gruppen-Callback (Mutex gehalten): Rohgruppe für die Konsole in die Queue, nie blockieren. */
static void on_group(const rds_group_t *g, void *ctx)
{
#if SHOW_RAW_GROUPS
    xQueueSend(s_grp_q, g, 0);
#endif
}

static void dsp_task(void *arg)
{
    /* --- 1. Abtastrate messen, ggf. nachstellen --- */
    ESP_ERROR_CHECK(adc_begin(s_req_rate_hz));
    ESP_LOGI(TAG, "Messe ADC-Abtastrate (%d s) ...", MEASURE_SECONDS);
    float fs = measure_rate();
    float res = RDS_CARRIER_HZ - fs * 0.25f;
    ESP_LOGI(TAG, "angefordert %u Hz, gemessen %.1f Hz → Restträger %.1f Hz",
             (unsigned)s_req_rate_hz, fs, res);

    if (fabsf(res) > RETUNE_THRESHOLD_HZ) {
        /* Der Treiber trifft die Rate nicht genau: Sollwert proportional korrigieren. */
        s_req_rate_hz = (uint32_t)((double)s_req_rate_hz * (228000.0 / (double)fs) + 0.5);
        ESP_LOGW(TAG, "Abweichung zu groß, stelle ADC auf %u Hz nach", (unsigned)s_req_rate_hz);
        adc_end();
        ESP_ERROR_CHECK(adc_begin(s_req_rate_hz));
        fs  = measure_rate();
        res = RDS_CARRIER_HZ - fs * 0.25f;
        ESP_LOGI(TAG, "neu gemessen %.1f Hz → Restträger %.1f Hz", fs, res);
    }
    s_fs_actual = fs;

    /* --- 2. Demodulator + Dekoder initialisieren --- */
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    rds_decoder_init(&s_dec, on_group, NULL);
    rds_dsp_init(&s_dsp, fs, on_bit, NULL);
    xSemaphoreGive(s_mtx);
    ESP_LOGI(TAG, "Decoder läuft.");

    /* --- 3. Hauptschleife: pro DMA-Frame ~1000 Samples verarbeiten --- */
    uint32_t last_ovf = 0;
    for (;;) {
        int n = adc_read_samples(200);
        if (n <= 0) continue;

        if (s_ovf_count != last_ovf) {                  /* Samples verloren gegangen */
            last_ovf = s_ovf_count;
            ESP_LOGW(TAG, "ADC-Pool-Überlauf (#%u) – Verarbeitung zu langsam", (unsigned)last_ovf);
        }

        xSemaphoreTake(s_mtx, portMAX_DELAY);
        s_now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        rds_dsp_process(&s_dsp, s_smp, (size_t)n);
        xSemaphoreGive(s_mtx);
    }
}

/* ============================== Konsole ============================== */

static void print_group(const rds_group_t *g)
{
    static const char names[4] = { 'A', 'B', 'C', 'D' };
    char blk[5];
    for (int i = 0; i < 4; i++) blk[i] = (g->ok & (1 << i)) ? ((g->corrected & (1 << i)) ? 'c' : names[i]) : '.';
    blk[4] = 0;

    if (g->ok & 2) {
        unsigned t = g->blk[1] >> 12, v = (g->blk[1] >> 11) & 1;
        printf("GRP %2u%c  %04X %04X %04X %04X  [%s]\n", t, v ? 'B' : 'A',
               g->blk[0], g->blk[1], g->blk[2], g->blk[3], blk);
    } else {
        printf("GRP  ??  %04X %04X %04X %04X  [%s]\n",
               g->blk[0], g->blk[1], g->blk[2], g->blk[3], blk);
    }
}

static void print_summary(uint32_t *last_groups, int64_t *last_t)
{
    /* Zustand unter Mutex in lokale Kopie ziehen, danach frei ausgeben */
    rds_info_t i;
    rds_dsp_status_t st;
    uint32_t now_ms;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    i = s_dec.info;
    rds_dsp_get_status(&s_dsp, &st);
    now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreGive(s_mtx);

    int64_t t = esp_timer_get_time();
    float rate = (float)(i.groups_ok - *last_groups) * 1.0e6f / (float)(t - *last_t);
    *last_groups = i.groups_ok; *last_t = t;

    char ps[40], rt[220], title[100], artist[100];
    rds_text_utf8(i.ps, RDS_PS_LEN, ps, sizeof ps);
    rds_text_utf8(i.rt, rds_rt_length(&i), rt, sizeof rt);
    bool has_title  = rds_rtp_text(&i, 1, title,  sizeof title);    /* ITEM.TITLE  */
    bool has_artist = rds_rtp_text(&i, 4, artist, sizeof artist);   /* ITEM.ARTIST */

    printf("\n──────────── RDS  (t = %lu s) ────────────\n", (unsigned long)(now_ms / 1000));
    printf(" Empfang    : %s | Costas %s (Δf %+.1f Hz, Lock %.2f) | Pegel %.0f | %.1f Gruppen/s\n",
           i.synced ? "SYNC" : "kein Sync", st.costas_locked ? "locked" : "sucht",
           st.carrier_offset_hz, st.lock_metric, st.signal_rms, rate);
    /* DC-Bias: automatisch erkannter Bias-Punkt in ADC-LSB (12 Bit, 0..4095).
     * Weicht er dauerhaft stark von der erwarteten Fenstermitte des gewählten
     * ADC_ATTEN ab (bei DB_12 ≈ 2048, bei DB_0 eher ~590 LSB für ~475 mV),
     * deutet das auf einen falsch dimensionierten Bias-Spannungsteiler hin. */
    printf(" DC-Bias    : %.0f LSB (automatisch erkannt)\n", st.dc_bias);
    if (i.pi_valid) printf(" Station    : PI %04X  PS \"%s\"", i.pi, ps);
    else            printf(" Station    : (PI unbekannt)");
    printf("  PTY %u  TP %u TA %u  %s\n", i.pty, i.tp, i.ta, i.ms ? "Musik" : "Sprache");

    if (has_title || has_artist)
        printf(" Titel (RT+): %s%s%s\n", has_artist ? artist : "",
               (has_artist && has_title) ? " – " : "", has_title ? title : "");
    if (i.rt_valid) printf(" RadioText  : \"%s\"\n", rt);
    else            printf(" RadioText  : (noch nichts empfangen)\n");

    rds_datetime_t dt;
    if (rds_local_time(&i, now_ms, &dt))
        printf(" Uhrzeit    : %04d-%02d-%02d %02d:%02d:%02d  (UTC%+d:%02d)\n", dt.year, dt.mon, dt.day,
               dt.hour, dt.min, dt.sec, dt.off_min / 60, abs(dt.off_min) % 60);
    else
        printf(" Uhrzeit    : (CT-Gruppe 4A noch nicht empfangen, kommt 1×/Minute)\n");

    printf(" Statistik  : Blöcke ok %lu / schlecht %lu / korrigiert %lu, Sync-Verluste %lu, fs %.1f Hz, Overruns %lu\n",
           (unsigned long)i.blocks_ok, (unsigned long)i.blocks_bad, (unsigned long)i.blocks_corrected,
           (unsigned long)i.sync_losses, s_fs_actual, (unsigned long)s_ovf_count);
    printf("──────────────────────────────────────────\n");
}

static void console_task(void *arg)
{
    uint32_t last_groups = 0;
    int64_t  last_t = esp_timer_get_time();
    int64_t  next   = last_t + (int64_t)PRINT_INTERVAL_MS * 1000;
    rds_group_t g;

    for (;;) {
        if (xQueueReceive(s_grp_q, &g, pdMS_TO_TICKS(50)) == pdTRUE) {
#if SHOW_RAW_GROUPS
            print_group(&g);
#endif
        }
        if (esp_timer_get_time() >= next) {
            next += (int64_t)PRINT_INTERVAL_MS * 1000;
            print_summary(&last_groups, &last_t);
        }
    }
}

/* ============================== app_main ============================== */
void app_main(void)
{
    /* WLAN/BT werden nirgends initialisiert → bleiben aus, kein Funkstörer am ADC. */
    ESP_LOGI(TAG, "RDS-Decoder, ADC1_CH6 (GPIO34), %d Hz", ADC_SAMPLE_RATE_HZ);

    s_mtx   = xSemaphoreCreateMutex();
    s_grp_q = xQueueCreate(64, sizeof(rds_group_t));
    configASSERT(s_mtx && s_grp_q);

    /* Konsole auf Core 0 (niedrige Prio), DSP auf Core 1 (hohe Prio, blockiert nur im ADC-Read) */
    xTaskCreatePinnedToCore(console_task, "rds_console", 6144, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(dsp_task,     "rds_dsp",     8192, NULL, 10, NULL, 1);
}
