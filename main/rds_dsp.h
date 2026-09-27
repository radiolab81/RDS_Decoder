/*
 * rds_dsp.h – RDS-Demodulator (MPX-Samples -> Bitstrom)
 *
 * Reine C-Bibliothek ohne ESP-IDF-Abhängigkeit (läuft auch auf dem PC-Host-Test).
 *
 * Signalkette (fs ≈ 228 kHz = 4 × 57 kHz):
 *
 *   ADC-Sample ─► Mischer 57 kHz (Folge +1,0,-1,0 / 0,-1,0,+1 → nur Vorzeichen)
 *              ─► CIC 4. Ordnung, Dezimierung ÷24  → 9,5 kS/s (= 8 Samples/Bit)
 *              ─► FIR-Tiefpass (41 Taps, ~2,4 kHz)
 *              ─► Costas-Loop (2. Ordnung, Fehler e = I·Q, normiert)
 *              ─► Manchester-Matched-Filter (Integrate & Dump, 8 Samples)
 *              ─► Bit-Timing (Early/Late-Schleife + grobe Phasensuche)
 *              ─► Differenzdekodierung  → Bit-Callback
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RDS_CIC_R        24        /* Dezimierungsfaktor: 228k / 24 = 9,5 kS/s   */
#define RDS_FIR_TAPS     41        /* Länge des Basisband-Tiefpasses              */
#define RDS_CARRIER_HZ   57000.0f  /* RDS-Unterträger (3 × Pilotton)             */
#define RDS_BITRATE_HZ   1187.5f   /* 57000 / 48                                  */

/* Wird für jedes dekodierte (differenz-dekodierte) Bit aufgerufen. */
typedef void (*rds_bit_cb_t)(uint8_t bit, void *ctx);

/* Statusinformationen für die Konsolenausgabe (nur lesen). */
typedef struct {
    bool  costas_locked;       /* Träger-Schleife eingerastet?                 */
    float carrier_offset_hz;   /* Restträger relativ zu fs/4 (vom NCO)         */
    float samples_per_bit;     /* aktuelle Bitlänge in 9,5-kHz-Samples (≈8)    */
    float signal_rms;          /* RDS-Signalpegel nach Filter, in ADC-LSB      */
    float lock_metric;         /* 0..1, ~0,9 bei sauberem Empfang              */
    float dc_bias;             /* automatisch erkannter DC-Bias, in ADC-LSB    */
} rds_dsp_status_t;

typedef struct {
    /* --- Konfiguration --- */
    float fs_in;               /* tatsächliche ADC-Abtastrate [Hz]             */
    float fs_out;              /* fs_in / RDS_CIC_R                            */
    rds_bit_cb_t cb;
    void *cb_ctx;

    /* --- DC-Offset-Tracking (automatische Bias-Erkennung) ---
     * Gleitender Mittelwert der Rohsamples in Q16-Festkomma, ersetzt die
     * früher fest codierte Konstante 2048. Läuft unabhängig davon, welchen
     * ADC_ATTEN-Bereich bzw. welchen realen Bias-Spannungsteiler man nutzt. */
    int32_t  dc_q16;                 /* aktueller Schätzwert, Q16 (Wert*65536) */
    bool     dc_init;                /* erstes Sample: Schätzwert direkt setzen */

    /* --- Mischer + CIC (Ganzzahl, Überlauf ist gewollt: Modulo-2^32) --- */
    uint8_t  mix_ph;                 /* Phase 0..3 der 57-kHz-Mischfolge       */
    uint8_t  dec_cnt;                /* Zähler bis zur Dezimierung             */
    uint32_t i_int[4], q_int[4];     /* Integratoren                           */
    uint32_t i_comb[4], q_comb[4];   /* Kamm-Verzögerungen                     */

    /* --- FIR-Tiefpass --- */
    float   fir_h[RDS_FIR_TAPS];
    float   fir_i[RDS_FIR_TAPS];
    float   fir_q[RDS_FIR_TAPS];
    uint8_t fir_pos;

    /* --- Costas-Loop --- */
    float phase;               /* NCO-Phase [rad]                              */
    float freq;                /* NCO-Frequenz [rad/Sample @ fs_out]           */
    float freq_center;         /* Startwert (aus Abtastraten-Messung)          */
    float kp, ki;              /* Schleifenkoeffizienten (aktuell)             */
    float agc;                 /* mittlere Leistung (I²+Q²) nach Derotation    */
    float lock_metric;
    bool  locked;

    /* --- Matched-Filter und Bit-Timing --- */
    float    rbuf[8];          /* letzte 8 derotierte I-Samples (1 Bit)        */
    uint8_t  rpos;
    float    m1, m2;           /* MF-Ausgang eine bzw. zwei Samples zuvor      */
    float    bit_timer;        /* Samples seit letzter Entscheidung            */
    float    spb, spb_nom;     /* Samples/Bit (aktuell / nominal)              */
    float    bins[8];          /* Energie je Phasenlage (grobe Synchronisation)*/
    uint32_t nsamp;
    uint16_t bit_cnt;
    bool     have_prev;
    uint8_t  prev_raw;
} rds_dsp_t;

/* fs_actual_hz: tatsächlich gemessene ADC-Rate (nominal 228000). Die Abweichung
 * zu 4×57 kHz wird als Startfrequenz in die Costas-Loop übernommen. */
void rds_dsp_init(rds_dsp_t *d, float fs_actual_hz, rds_bit_cb_t cb, void *ctx);

/* Verarbeitet n rohe 12-Bit-ADC-Werte (0..4095). */
void rds_dsp_process(rds_dsp_t *d, const uint16_t *samples, size_t n);

void rds_dsp_get_status(const rds_dsp_t *d, rds_dsp_status_t *st);

#ifdef __cplusplus
}
#endif
