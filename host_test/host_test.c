/*
 * host_test.c – lässt DSP + Gruppendekoder auf dem PC über eine Sample-Datei laufen.
 *
 *   gcc -O2 -I../main host_test.c ../main/rds_dsp.c ../main/rds_group.c -lm -o host_test
 *   ./host_test mpx.bin [fs_hz=228000] [v]      (v = jede Gruppe ausgeben)
 */
#include <stdio.h>
#include <stdlib.h>
#include "rds_dsp.h"
#include "rds_group.h"

static rds_decoder_t dec;
static uint32_t now_ms;
static unsigned long nbits;

static void on_group(const rds_group_t *g, void *ctx)
{   /* Rohgruppen ausgeben (nur mit drittem Argument "v") */
    (void)ctx;
    printf("GRP %04X %04X %04X %04X [%c%c%c%c]\n", g->blk[0], g->blk[1], g->blk[2], g->blk[3],
           (g->ok&1)?'A':'.', (g->ok&2)?'B':'.', (g->ok&4)?'C':'.', (g->ok&8)?'D':'.');
}

static void on_bit(uint8_t bit, void *ctx) { (void)ctx; nbits++; rds_decoder_push_bit(&dec, bit, now_ms); }

int main(int argc, char **argv)
{
    if (argc < 2) return 1;
    float fs = (argc > 2) ? (float)atof(argv[2]) : 228000.0f;
    FILE *f = fopen(argv[1], "rb"); if (!f) { perror("open"); return 1; }
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    size_t n = (size_t)sz / 2;
    uint16_t *buf = malloc((size_t)sz); if (fread(buf, 2, n, f) != n) return 1; fclose(f);

    static rds_dsp_t dsp;
    rds_decoder_init(&dec, (argc > 3) ? on_group : NULL, NULL);
    rds_dsp_init(&dsp, fs, on_bit, NULL);

    const size_t FR = 1024;
    for (size_t p = 0; p < n; p += FR) {
        now_ms = (uint32_t)((double)p / fs * 1000.0);
        rds_dsp_process(&dsp, buf + p, (n - p < FR) ? n - p : FR);
    }
    now_ms = (uint32_t)((double)n / fs * 1000.0);

    rds_dsp_status_t st; rds_dsp_get_status(&dsp, &st);
    const rds_info_t *i = &dec.info;
    char ps[32], rt[128], ti[64] = "-", ar[64] = "-";
    rds_text_utf8(i->ps, RDS_PS_LEN, ps, sizeof ps);
    rds_text_utf8(i->rt, rds_rt_length(i), rt, sizeof rt);
    rds_rtp_text(i, 1, ti, sizeof ti); rds_rtp_text(i, 4, ar, sizeof ar);
    rds_datetime_t dt = {0}; bool hasdt = rds_local_time(i, now_ms, &dt);

    printf("bits=%lu  locked=%d  df=%.2f Hz  spb=%.4f  rms=%.1f lock=%.2f\n",
           nbits, st.costas_locked, st.carrier_offset_hz, st.samples_per_bit, st.signal_rms, st.lock_metric);
    printf("sync=%d PI=%04X PTY=%u PS=\"%s\" RT=\"%s\"\n", i->synced, i->pi, i->pty, ps, rt);
    printf("RT+ title=\"%s\" artist=\"%s\"\n", ti, ar);
    if (hasdt) printf("time %04d-%02d-%02d %02d:%02d:%02d (off %d min)\n", dt.year, dt.mon, dt.day, dt.hour, dt.min, dt.sec, dt.off_min);
    printf("groups=%u blocks ok=%u bad=%u corrected=%u sync_losses=%u\n",
           i->groups_ok, i->blocks_ok, i->blocks_bad, i->blocks_corrected, i->sync_losses);
    return 0;
}
