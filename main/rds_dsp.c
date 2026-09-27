/*
 * rds_dsp.c – RDS-Demodulator, siehe rds_dsp.h für den Überblick.
 */
#include "rds_dsp.h"

#include <math.h>
#include <string.h>

#define TWO_PI_F      6.28318530718f
#define PI_F          3.14159265359f

/* CIC 4. Ordnung: In den I- bzw. Q-Zweig gehen nur je die HÄLFTE der Samples
 * (die anderen sind 0), pro Ausgabewert also 12 Werte → Gewinn 12^4.        */
#define CIC_GAIN_INV  (1.0f / (12.0f * 12.0f * 12.0f * 12.0f))

/* Costas-Bandbreiten (Einseitenrauschbandbreite Bn in Hz) */
#define COSTAS_BN_ACQ_HZ    60.0f   /* Einfangen: breit, schnell               */
#define COSTAS_BN_TRK_HZ    20.0f   /* Nachführen: schmal, rauscharm           */
#define COSTAS_MAX_DEV_HZ   150.0f  /* NCO darf max. so weit vom Startwert weg */

/* Bit-Timing-Schleife */
#define TIM_KP              0.03f   /* proportional [Samples pro Bit]          */
#define TIM_KI              2.0e-4f /* integral (folgt Taktabweichung)         */

/* -------------------------------------------------------------------------
 * DC-Bias-Tracking (ersetzt die feste Konstante "2048").
 *
 * Der reale Bias-Punkt hängt vom gewählten ADC_ATTEN (0 dB: Fenstermitte um
 * ~475–500 mV statt 1,65 V bei 12 dB) und von Bauteiltoleranzen/Drift des
 * Spannungsteilers ab. Ein einzelner Konstantwert im Code trifft das nur
 * zufällig. Statt dessen wird der Mittelwert der Rohsamples per einfachem
 * IIR-Tiefpass 1. Ordnung laufend geschätzt:
 *
 *   dc[n] = dc[n-1] + (x[n] - dc[n-1]) >> DC_SHIFT
 *
 * Zeitkonstante: tau ≈ 2^DC_SHIFT / fs_in. Mit DC_SHIFT=14 und fs_in≈228 kHz
 * ergibt das tau ≈ 72 ms (Grenzfrequenz ≈ 2,2 Hz) – weit unter dem 19-kHz-
 * Pilotton, sodass kein Nutzsignal "weggeregelt" wird, aber schnell genug,
 * um Temperaturdrift und den Einschwingvorgang nach dem Start zu erfassen.
 * Rechnung in Q16-Festkomma (kein float im ISR-kritischen Vollraten-Pfad). */
#define DC_SHIFT   14
#define DC_Q16(x)  ((int32_t)(x) << 16)

/* -------------------------------------------------------------------------
 * Costas-Koeffizienten aus der gewünschten Rauschbandbreite (ζ = 0,707).
 * Detektorverstärkung ist durch die AGC-Normierung = 1.
 * ------------------------------------------------------------------------- */
static void costas_set_bw(rds_dsp_t *d, float bn_hz)
{
    const float zeta = 0.7071f;
    float bnt = bn_hz / d->fs_out;                   /* Bn·T               */
    float th  = bnt / (zeta + 0.25f / zeta);
    float den = 1.0f + 2.0f * zeta * th + th * th;
    d->kp = 4.0f * zeta * th / den;
    d->ki = 4.0f * th * th / den;
}

void rds_dsp_init(rds_dsp_t *d, float fs_actual_hz, rds_bit_cb_t cb, void *ctx)
{
    memset(d, 0, sizeof(*d));
    d->cb = cb;
    d->cb_ctx = ctx;
    d->fs_in  = fs_actual_hz;
    d->fs_out = fs_actual_hz / (float)RDS_CIC_R;

    /* Der Mischer läuft fest bei fs/4. Ist fs nicht exakt 228 kHz, liegt der
     * RDS-Träger im Basisband bei +res Hz. Diese Frequenz geben wir dem NCO
     * gleich als Startwert mit, dann muss die Schleife nur noch Feinarbeit
     * leisten.                                                              */
    float res_hz   = RDS_CARRIER_HZ - fs_actual_hz * 0.25f;
    d->freq        = TWO_PI_F * res_hz / d->fs_out;
    d->freq_center = d->freq;
    costas_set_bw(d, COSTAS_BN_ACQ_HZ);
    d->agc = 1.0f;

    /* Bitlänge in Basisband-Samples: nominal 8, bei Abweichung der ADC-Rate
     * entsprechend anders. Die Timing-Schleife trimmt den Rest nach.        */
    d->spb_nom = d->fs_out / RDS_BITRATE_HZ;
    d->spb     = d->spb_nom;

    /* DC-Schätzwert startet neutral bei 2048 (Mitte des 12-Bit-ADC-Bereichs)
     * und wird beim allerersten verarbeiteten Sample sofort auf dessen
     * tatsächlichen Wert gesprungen (dc_init), damit das Einschwingen nicht
     * 72 ms lang von einem womöglich falschen Startwert aus erfolgen muss. */
    d->dc_q16 = DC_Q16(2048);
    d->dc_init = false;

    /* Tiefpass: gefenstertes Sinc (Hamming), Grenzfrequenz 2,4 kHz.
     * Das Manchester-Spektrum (Nullstelle bei 2×1187,5 Hz) liegt darunter,
     * der untere L−R-Rand (4 kHz Abstand) wird unterdrückt.                 */
    const int   M  = (RDS_FIR_TAPS - 1) / 2;
    const float fc = 2400.0f / d->fs_out;            /* normiert (Zyklen/Sample) */
    float sum = 0.0f;
    for (int k = 0; k < RDS_FIR_TAPS; k++) {
        float n = (float)(k - M);
        float s = (n == 0.0f) ? 2.0f * fc
                              : sinf(TWO_PI_F * fc * n) / (PI_F * n);
        float w = 0.54f - 0.46f * cosf(TWO_PI_F * (float)k / (float)(RDS_FIR_TAPS - 1));
        d->fir_h[k] = s * w;
        sum += d->fir_h[k];
    }
    for (int k = 0; k < RDS_FIR_TAPS; k++) d->fir_h[k] /= sum;   /* DC-Gain = 1 */
}

/* -------------------------------------------------------------------------
 * Verarbeitung eines Basisband-Samples (9,5 kS/s), yi/yq aus dem CIC.
 * ------------------------------------------------------------------------- */
static void baseband_sample(rds_dsp_t *d, float yi, float yq)
{
    /* ---- 1. FIR-Tiefpass (I und Q getrennt) ---- */
    d->fir_i[d->fir_pos] = yi;
    d->fir_q[d->fir_pos] = yq;
    float fi = 0.0f, fq = 0.0f;
    int p = d->fir_pos;
    for (int k = 0; k < RDS_FIR_TAPS; k++) {
        fi += d->fir_h[k] * d->fir_i[p];
        fq += d->fir_h[k] * d->fir_q[p];
        p = (p == 0) ? RDS_FIR_TAPS - 1 : p - 1;
    }
    d->fir_pos = (uint8_t)((d->fir_pos + 1) % RDS_FIR_TAPS);

    /* ---- 2. Costas-Loop: Derotation um -phase ---- */
    float c = cosf(d->phase), s = sinf(d->phase);
    float ri = fi * c + fq * s;          /* Realteil von (fi + j·fq)·e^(-jφ)   */
    float rq = fq * c - fi * s;          /* Imaginärteil                       */

    /* AGC = gleitender Mittelwert der Leistung; dient nur zur Normierung
     * des Phasenfehlers, damit die Schleifenbandbreite pegelunabhängig ist. */
    float pwr = ri * ri + rq * rq;
    d->agc += (pwr - d->agc) * (1.0f / 1024.0f);

    /* Phasenfehler des BPSK-Costas: e = I·Q / P  ≈ (1/2)·sin(2φ) ≈ φ.
     * Unempfindlich gegen die 180°-Mehrdeutigkeit (Datenvorzeichen).        */
    float e = (ri * rq) / (d->agc + 1.0e-3f);
    if (e >  1.0f) e =  1.0f;
    if (e < -1.0f) e = -1.0f;

    /* PI-Filter: Integrator = Frequenz, proportionaler Anteil = Phase */
    d->freq  += d->ki * e;
    float dev = TWO_PI_F * COSTAS_MAX_DEV_HZ / d->fs_out;
    if (d->freq > d->freq_center + dev) d->freq = d->freq_center + dev;
    if (d->freq < d->freq_center - dev) d->freq = d->freq_center - dev;
    d->phase += d->freq + d->kp * e;
    if (d->phase >  PI_F) d->phase -= TWO_PI_F;
    if (d->phase < -PI_F) d->phase += TWO_PI_F;

    /* Lock-Detektor: Bei Einrasten liegt die Energie im I-Zweig,
     * (I²−Q²)/(I²+Q²) → ~1. Bei reinem Rauschen ~0. Mit Hysterese.          */
    float lm = (pwr > 1.0e-6f) ? (ri * ri - rq * rq) / pwr : 0.0f;
    d->lock_metric += (lm - d->lock_metric) * (1.0f / 512.0f);
    if (!d->locked && d->lock_metric > 0.70f) {
        d->locked = true;
        costas_set_bw(d, COSTAS_BN_TRK_HZ);      /* schmal schalten          */
    } else if (d->locked && d->lock_metric < 0.30f) {
        d->locked = false;
        costas_set_bw(d, COSTAS_BN_ACQ_HZ);      /* wieder breit             */
    }

    /* ---- 3. Manchester-Matched-Filter (8 Samples = 1 Bit) ----
     * Korrelation mit Schablone (+,+,+,+,-,-,-,-): erste Bithälfte minus
     * zweite Bithälfte. Läuft gleitend auf jedem Sample.                    */
    d->rbuf[d->rpos] = ri;
    d->rpos = (d->rpos + 1) & 7;
    float mf = 0.0f;
    for (int k = 0; k < 4; k++) {
        mf += d->rbuf[(d->rpos + k) & 7];         /* ältere Hälfte  (+)      */
        mf -= d->rbuf[(d->rpos + 4 + k) & 7];     /* jüngere Hälfte (−)      */
    }

    /* ---- 4. Bit-Timing ----
     * a) grobe Suche: leaky Energie |mf| je Phasenlage (0..7).
     * b) fein: Early/Late-Vergleich um den Entscheidungszeitpunkt.
     * Entscheidungszeitpunkt ist das VORLETZTE MF-Sample (m1); m2 = early,
     * mf = late.                                                            */
    unsigned idx0 = d->nsamp & 7u;
    d->nsamp++;
    d->bins[idx0] += (fabsf(mf) - d->bins[idx0]) * (1.0f / 64.0f);

    d->bit_timer += 1.0f;
    if (d->bit_timer >= d->spb) {
        float a0 = fabsf(mf), a1 = fabsf(d->m1), a2 = fabsf(d->m2);
        uint8_t raw = (d->m1 > 0.0f) ? 1 : 0;      /* Manchester-Symbol      */

        /* Early/Late-Fehler: >0 → Maximum liegt später als unser Punkt.     */
        float err = (a0 - a2) / (a0 + a1 + a2 + 1.0e-3f);
        d->spb += TIM_KI * err;
        if (d->spb > d->spb_nom * 1.02f) d->spb = d->spb_nom * 1.02f;
        if (d->spb < d->spb_nom * 0.98f) d->spb = d->spb_nom * 0.98f;
        d->bit_timer -= d->spb + TIM_KP * err;

        /* Grobkorrektur: Liegt das Energiemaximum ≥2 Samples neben unserem
         * Entscheidungspunkt (z. B. direkt nach dem Start), springen wir hin. */
        if ((++d->bit_cnt & 31u) == 0) {
            unsigned cur = (idx0 + 7u) & 7u;       /* Phase des Entscheidungspunkts */
            unsigned best = 0;
            for (unsigned k = 1; k < 8; k++) if (d->bins[k] > d->bins[best]) best = k;
            int dd = (int)((best - cur + 4u) & 7u) - 4;   /* −4..+3         */
            if (dd >= 2 || dd <= -2) d->bit_timer -= (float)dd;
        }

        /* Differenzdekodierung: RDS überträgt d[n] = a[n] XOR d[n−1].
         * Dadurch ist die 180°-Mehrdeutigkeit des Costas-Loops egal.        */
        if (d->have_prev && d->cb) d->cb((uint8_t)(raw ^ d->prev_raw), d->cb_ctx);
        d->prev_raw  = raw;
        d->have_prev = true;
    }
    d->m2 = d->m1;
    d->m1 = mf;
}

/* -------------------------------------------------------------------------
 * Hauptschleife bei voller Abtastrate (~228 kS/s): nur Additionen.
 * ------------------------------------------------------------------------- */
void rds_dsp_process(rds_dsp_t *d, const uint16_t *s, size_t n)
{
    for (size_t k = 0; k < n; k++) {
        int32_t raw = (int32_t)s[k];

        /* --- automatische DC-Bias-Erkennung --- */
        if (!d->dc_init) { d->dc_q16 = DC_Q16(raw); d->dc_init = true; }  /* Sprungstart */
        d->dc_q16 += (DC_Q16(raw) - d->dc_q16) >> DC_SHIFT;               /* IIR-Nachführung */
        int32_t dc = d->dc_q16 >> 16;                                    /* zurück auf LSB-Skala */

        int32_t x = raw - dc;                      /* jetzt bias-frei statt "- 2048" */

        /* Mischen mit cos/sin(π·n/2): I = x·(+1,0,−1,0), Q = −x·sin = (0,−1,0,+1)·x.
         * Ergebnis: Signal bei +57 kHz landet bei 0 Hz (+ Restfrequenz).      */
        uint32_t iin = 0, qin = 0;
        switch (d->mix_ph) {
            case 0:  iin = (uint32_t)x;    break;
            case 1:  qin = (uint32_t)(-x); break;
            case 2:  iin = (uint32_t)(-x); break;
            default: qin = (uint32_t)x;    break;
        }
        d->mix_ph = (d->mix_ph + 1) & 3;

        /* CIC-Integratoren (4 Stufen je Zweig) */
        d->i_int[0] += iin;           d->q_int[0] += qin;
        d->i_int[1] += d->i_int[0];   d->q_int[1] += d->q_int[0];
        d->i_int[2] += d->i_int[1];   d->q_int[2] += d->q_int[1];
        d->i_int[3] += d->i_int[2];   d->q_int[3] += d->q_int[2];

        if (++d->dec_cnt == RDS_CIC_R) {           /* Dezimierung ÷24        */
            d->dec_cnt = 0;

            /* Kammstufen (Differenz zum vorigen Wert) */
            uint32_t vi = d->i_int[3], vq = d->q_int[3], t;
            for (int st = 0; st < 4; st++) {
                t = vi - d->i_comb[st]; d->i_comb[st] = vi; vi = t;
                t = vq - d->q_comb[st]; d->q_comb[st] = vq; vq = t;
            }
            baseband_sample(d, (float)(int32_t)vi * CIC_GAIN_INV,
                               (float)(int32_t)vq * CIC_GAIN_INV);
        }
    }
}

void rds_dsp_get_status(const rds_dsp_t *d, rds_dsp_status_t *st)
{
    st->costas_locked     = d->locked;
    st->carrier_offset_hz = d->freq * d->fs_out / TWO_PI_F;
    st->samples_per_bit   = d->spb;
    st->signal_rms        = sqrtf(d->agc);
    st->lock_metric       = d->lock_metric;
    st->dc_bias           = (float)(d->dc_q16 >> 16);
}
