/*
 * rds_group.c – Blocksynchronisation, CRC, Gruppendekodierung (EN 50067 / IEC 62106)
 *
 * Aufbau des RDS-Datenstroms:
 *   Gruppe = 4 Blöcke × 26 Bit = 104 Bit  (≈ 11,4 Gruppen/s bei 1187,5 Bit/s)
 *   Block  = 16 Bit Information + 10 Bit Checkwort (CRC, mit "Offset-Wort" verknüpft)
 *   Die Offset-Wörter A, B, C (oder C'), D kennzeichnen die Blockposition und
 *   erlauben es, ohne Rahmenmarke zu synchronisieren.
 */
#include "rds_group.h"

#include <string.h>

/* Generatorpolynom g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1 */
#define RDS_POLY 0x5B9u

/* Offset-Wörter. Bei fehlerfreiem Block ist der CRC-Rest genau dieses Wort. */
enum { OFF_A = 0, OFF_B, OFF_C, OFF_D, OFF_CP, OFF_COUNT };
static const uint16_t k_offset[OFF_COUNT] = { 0x0FC, 0x198, 0x168, 0x1B4, 0x350 };

/* Rest der Polynomdivision (GF(2)) des 26-Bit-Blocks durch g(x). */
static uint16_t rds_syndrome(uint32_t blk26)
{
    uint32_t r = blk26;
    for (int i = 25; i >= 10; i--)
        if (r & (1u << i)) r ^= (RDS_POLY << (i - 10));
    return (uint16_t)(r & 0x3FF);
}

void rds_decoder_init(rds_decoder_t *d, rds_group_cb_t on_group, void *ctx)
{
    memset(d, 0, sizeof(*d));
    d->on_group = on_group;
    d->ctx      = ctx;
    d->last_off = -1;
    d->info.rtp_gtype = 0xFF;
    memset(d->info.ps, ' ', RDS_PS_LEN);
    memset(d->info.rt, ' ', RDS_RT_LEN);
}

/* -------------------------------------------------------------------------
 * Bündelfehler-Korrektur: probiert 1- und 2-Bit-Bündel an jeder Position.
 * (Der Code könnte mehr, aber je mehr man korrigiert, desto höher die
 *  Gefahr von Fehlkorrekturen – 2 Bit ist ein konservativer Kompromiss.)
 * ------------------------------------------------------------------------- */
static bool try_correct(uint32_t reg, uint16_t offset_word, uint32_t *fixed)
{
    for (int len = 1; len <= 2; len++) {
        uint32_t mask = (1u << len) - 1u;
        for (int pos = 0; pos + len <= 26; pos++) {
            uint32_t cand = reg ^ (mask << pos);
            if (rds_syndrome(cand) == offset_word) { *fixed = cand; return true; }
        }
    }
    return false;
}

/* =========================================================================
 *  Gruppendekodierung
 * ========================================================================= */

static void ps_store(rds_decoder_t *d, int seg, uint8_t c0, uint8_t c1, bool corrected)
{
    uint8_t m = (uint8_t)(1u << seg);
    if (corrected) {
        /* Korrigierte Segmente erst übernehmen, wenn sie ein zweites Mal
         * identisch ankommen (schützt vor Fehlkorrekturen).                 */
        if (!(d->ps_pend_mask & m) || d->ps_pend[seg * 2] != c0 || d->ps_pend[seg * 2 + 1] != c1) {
            d->ps_pend[seg * 2] = c0; d->ps_pend[seg * 2 + 1] = c1;
            d->ps_pend_mask |= m;
            return;
        }
    }
    d->info.ps[seg * 2] = c0; d->info.ps[seg * 2 + 1] = c1;
    d->info.ps_seen |= m;
    d->ps_pend_mask &= (uint8_t)~m;
}

static void rt_store(rds_decoder_t *d, int pos, const uint8_t *ch, int n, bool corrected)
{
    uint64_t m = 0;
    for (int k = 0; k < n; k++) m |= 1ull << (pos + k);
    if (corrected) {
        if ((d->rt_pend_mask & m) != m || memcmp(&d->rt_pend[pos], ch, (size_t)n) != 0) {
            memcpy(&d->rt_pend[pos], ch, (size_t)n);
            d->rt_pend_mask |= m;
            return;
        }
    }
    memcpy(&d->info.rt[pos], ch, (size_t)n);
    d->info.rt_seen |= m;
    d->info.rt_valid = true;
    d->rt_pend_mask &= ~m;
}

/* Gruppe 4A: Uhrzeit und Datum (CT), einmal pro Minute */
static void decode_ct(rds_decoder_t *d, const rds_group_t *g, uint32_t now_ms)
{
    uint16_t B = g->blk[1], C = g->blk[2], D = g->blk[3];
    int32_t mjd  = ((int32_t)(B & 3) << 15) | (C >> 1);            /* 17 Bit MJD */
    int     hour = ((C & 1) << 4) | (D >> 12);
    int     min  = (D >> 6) & 0x3F;
    int     off  = D & 0x1F;                                       /* Halbstunden */
    if (D & 0x20) off = -off;
    if (hour > 23 || min > 59) return;                             /* Plausibilität */
    rds_info_t *i = &d->info;
    i->ct_mjd = mjd; i->ct_hour = (uint8_t)hour; i->ct_min = (uint8_t)min;
    i->ct_off_min = (int16_t)(off * 30);
    i->ct_rx_ms = now_ms;
    i->ct_valid = true;
}

/* RT+ (ODA, AID 0x4BD7): Tags verweisen auf Start/Länge im RadioText */
static void decode_rtplus(rds_decoder_t *d, const rds_group_t *g)
{
    uint16_t B = g->blk[1], C = g->blk[2], D = g->blk[3];
    rds_info_t *i = &d->info;
    /* Bitlayout: B[4]=Toggle, B[3]=Running, B[2:0]=Typ1[5:3]
     *            C: Typ1[2:0] | Start1(6) | Len1(6) | Typ2[5]
     *            D: Typ2[4:0] | Start2(6) | Len2(5)                          */
    i->rtp_running = (B >> 3) & 1;
    i->rtp_tag[0].type  = (uint8_t)(((B & 7) << 3) | (C >> 13));
    i->rtp_tag[0].start = (C >> 7) & 0x3F;
    i->rtp_tag[0].len   = (uint8_t)(((C >> 1) & 0x3F) + 1);
    i->rtp_tag[1].type  = (uint8_t)(((C & 1) << 5) | (D >> 11));
    i->rtp_tag[1].start = (D >> 5) & 0x3F;
    i->rtp_tag[1].len   = (uint8_t)((D & 0x1F) + 1);
    i->rtp_valid = true;
}

static void decode_group(rds_decoder_t *d, const rds_group_t *g, uint32_t now_ms)
{
    rds_info_t *i = &d->info;
    bool okA = g->ok & 1, okB = g->ok & 2, okC = g->ok & 4, okD = g->ok & 8;

    if (okA) { i->pi = g->blk[0]; i->pi_valid = true; }
    if (!okB) return;                          /* ohne Block B kein Gruppentyp */

    uint16_t B = g->blk[1];
    unsigned type = B >> 12;
    unsigned ver  = (B >> 11) & 1;             /* 0 = A, 1 = B                */
    i->tp  = (B >> 10) & 1;
    i->pty = (B >> 5) & 0x1F;
    i->groups_ok++;

    bool fec_used = (g->corrected & g->ok) != 0;

    switch (type) {
    case 0: {                                   /* 0A/0B: Programme Service Name */
        if (!okD) break;
        int seg = B & 3;
        i->ta = (B >> 4) & 1;
        i->ms = (B >> 3) & 1;
        ps_store(d, seg, g->blk[3] >> 8, g->blk[3] & 0xFF, (g->corrected & 8) != 0);
        break;
    }
    case 2: {                                   /* 2A/2B: RadioText            */
        uint8_t ab = (B >> 4) & 1;
        int addr = B & 0xF;
        if (i->rt_valid && ab != i->rt_ab) {    /* Textwechsel → Puffer leeren */
            memset(i->rt, ' ', RDS_RT_LEN);
            i->rt_seen = 0; d->rt_pend_mask = 0;
        }
        i->rt_ab = ab;
        if (ver == 0) {                         /* 2A: 4 Zeichen (C + D)       */
            if (!okC || !okD) break;
            uint8_t ch[4] = { g->blk[2] >> 8, g->blk[2] & 0xFF, g->blk[3] >> 8, g->blk[3] & 0xFF };
            rt_store(d, addr * 4, ch, 4, (g->corrected & 12) != 0);
        } else {                                /* 2B: 2 Zeichen (nur D)       */
            if (!okD) break;
            uint8_t ch[2] = { g->blk[3] >> 8, g->blk[3] & 0xFF };
            rt_store(d, addr * 2, ch, 2, (g->corrected & 8) != 0);
        }
        break;
    }
    case 3:                                     /* 3A: ODA-Zuordnung (AID)     */
        if (ver == 0 && okD && g->blk[3] == 0x4BD7)
            i->rtp_gtype = B & 0x1F;            /* in welcher Gruppe kommt RT+? */
        break;
    case 4:                                     /* 4A: Uhrzeit                 */
        if (ver == 0 && okC && okD && !fec_used) decode_ct(d, g, now_ms);
        break;
    default: break;
    }

    /* RT+-Nutzdaten kommen in der bei 3A angekündigten Gruppe */
    if (i->rtp_gtype != 0xFF && ((type << 1) | ver) == i->rtp_gtype && okC && okD && !fec_used)
        decode_rtplus(d, g);
}

/* =========================================================================
 *  Blockweise Verarbeitung im synchronisierten Zustand
 * ========================================================================= */
static void process_block(rds_decoder_t *d, uint32_t now_ms)
{
    int idx = d->blk_idx;
    uint16_t syn = rds_syndrome(d->reg);
    uint32_t reg = d->reg;
    bool ok = false, fixed = false, cprime = false;

    /* Kandidaten für das erwartete Offset-Wort */
    int cand[2], nc = 0;
    if (idx == 2) {
        if (d->cur.ok & 2) cand[nc++] = ((d->cur.blk[1] >> 11) & 1) ? OFF_CP : OFF_C;
        else { cand[nc++] = OFF_C; cand[nc++] = OFF_CP; }   /* B unbekannt → beide */
    } else {
        cand[nc++] = idx;                                   /* A,B,D = 0,1,3 */
        if (idx == 3) cand[0] = OFF_D;
    }
    for (int k = 0; k < nc && !ok; k++) {
        if (syn == k_offset[cand[k]]) { ok = true; cprime = (cand[k] == OFF_CP); }
    }
    if (!ok) {                                              /* FEC versuchen   */
        for (int k = 0; k < nc && !ok; k++) {
            uint32_t f;
            if (try_correct(reg, k_offset[cand[k]], &f)) {
                ok = fixed = true; reg = f; cprime = (cand[k] == OFF_CP);
            }
        }
    }

    if (ok) {
        d->cur.blk[idx] = (uint16_t)(reg >> 10);
        d->cur.ok |= (uint8_t)(1u << idx);
        if (fixed) { d->cur.corrected |= (uint8_t)(1u << idx); d->info.blocks_corrected++; }
        if (idx == 2) d->cur.c_prime = cprime;
        d->info.blocks_ok++;
    } else {
        d->info.blocks_bad++;
    }

    /* Fehlerfenster: >35 schlechte von 50 Blöcken → Synchronisation verloren */
    d->win_n++;
    if (!ok) d->win_bad++;
    if (d->win_n >= 50) {
        if (d->win_bad > 35) {
            d->synced = false; d->info.synced = false;
            d->last_off = -1; d->since_valid = 0;
            d->info.sync_losses++;
        }
        d->win_n = d->win_bad = 0;
    }

    if (idx == 3) {                                         /* Gruppe komplett */
        if (d->cur.ok) {
            if (d->on_group) d->on_group(&d->cur, d->ctx);
            decode_group(d, &d->cur, now_ms);
        }
        memset(&d->cur, 0, sizeof(d->cur));
    }
    d->blk_idx  = (idx + 1) & 3;
    d->blk_bits = 0;
}

/* Index des Blocks (0..3) zu einem Offset-Wort */
static int block_index(int off) { return (off == OFF_CP) ? 2 : off; }

/* Ist "off" die erwartete Folge zu "prev"? A→B→C/C'→D→A */
static bool follows(int prev, int off)
{
    return block_index(off) == ((block_index(prev) + 1) & 3);
}

void rds_decoder_push_bit(rds_decoder_t *d, uint8_t bit, uint32_t now_ms)
{
    d->reg = ((d->reg << 1) | (bit & 1u)) & 0x3FFFFFFu;

    if (!d->synced) {
        /* Suchmodus: bei jedem Bit prüfen, ob die letzten 26 Bit ein gültiger
         * Block sind. Synchron, sobald zwei gültige Blöcke im Abstand von
         * genau 26 Bit in der richtigen Reihenfolge auftreten.              */
        if (d->since_valid < 100) d->since_valid++;
        uint16_t syn = rds_syndrome(d->reg);
        int off = -1;
        for (int k = 0; k < OFF_COUNT; k++) if (syn == k_offset[k]) { off = k; break; }
        if (off >= 0) {
            if (d->last_off >= 0 && d->since_valid == 26 && follows(d->last_off, off)) {
                d->synced = true; d->info.synced = true;
                d->win_n = d->win_bad = 0;
                memset(&d->cur, 0, sizeof(d->cur));
                d->blk_idx = block_index(off);
                process_block(d, now_ms);                 /* aktuellen Block sofort verbuchen */
                return;
            }
            d->last_off = off;
            d->since_valid = 0;
        }
        return;
    }

    if (++d->blk_bits == 26) process_block(d, now_ms);
}

/* =========================================================================
 *  Ausgabehilfen
 * ========================================================================= */

/* RDS-Zeichensatz (EBU Latin, Basissatz): ASCII + die im deutschen Sprachraum
 * üblichen Sonderzeichen. Alles andere wird zu '?'.                           */
static const char *rds_char_utf8(uint8_t c, char *tmp)
{
    switch (c) {
    case 0x80: return "á"; case 0x81: return "à"; case 0x82: return "é"; case 0x83: return "è";
    case 0x84: return "í"; case 0x85: return "ì"; case 0x86: return "ó"; case 0x87: return "ò";
    case 0x88: return "ú"; case 0x89: return "ù"; case 0x8A: return "Ñ"; case 0x8B: return "Ç";
    case 0x8D: return "ß";
    case 0x90: return "â"; case 0x91: return "ä"; case 0x92: return "ê"; case 0x93: return "ë";
    case 0x94: return "î"; case 0x95: return "ï"; case 0x96: return "ô"; case 0x97: return "ö";
    case 0x98: return "û"; case 0x99: return "ü"; case 0x9A: return "ñ"; case 0x9B: return "ç";
    case 0xC2: return "É"; case 0xD1: return "Ä"; case 0xD7: return "Ö"; case 0xD9: return "Ü";
    case 0xA1: return "α"; case 0xA9: return "£"; case 0xAB: return "€"; case 0xB0: return "º";
    case 0x24: return "$";                       /* im Standard '¤', in der Praxis '$' */
    default: break;
    }
    if (c >= 0x20 && c < 0x7F) { tmp[0] = (char)c; tmp[1] = 0; return tmp; }
    return "?";
}

size_t rds_text_utf8(const uint8_t *src, size_t len, char *dst, size_t dst_sz)
{
    size_t o = 0; char tmp[2];
    if (dst_sz == 0) return 0;
    for (size_t k = 0; k < len; k++) {
        const char *s = rds_char_utf8(src[k], tmp);
        size_t sl = strlen(s);
        if (o + sl + 1 > dst_sz) break;
        memcpy(dst + o, s, sl); o += sl;
    }
    dst[o] = 0;
    return o;
}

size_t rds_rt_length(const rds_info_t *i)
{
    size_t n = RDS_RT_LEN;
    for (size_t k = 0; k < RDS_RT_LEN; k++) if (i->rt[k] == 0x0D) { n = k; break; }
    while (n > 0 && i->rt[n - 1] == ' ') n--;
    return n;
}

bool rds_rtp_text(const rds_info_t *i, uint8_t content_type, char *out, size_t out_sz)
{
    if (!i->rtp_valid || !i->rt_valid) return false;
    for (int t = 0; t < 2; t++) {
        const rds_rtp_tag_t *g = &i->rtp_tag[t];
        if (g->type != content_type || g->len == 0) continue;
        if (g->start + g->len > RDS_RT_LEN) return false;
        for (int k = 0; k < g->len; k++)                    /* alle Zeichen schon empfangen? */
            if (!((i->rt_seen >> (g->start + k)) & 1u)) return false;
        rds_text_utf8(&i->rt[g->start], g->len, out, out_sz);
        return true;
    }
    return false;
}

/* MJD → Kalenderdatum nach EN 50067 Annex G, danach Offset + verstrichene Zeit */
bool rds_local_time(const rds_info_t *i, uint32_t now_ms, rds_datetime_t *o)
{
    if (!i->ct_valid) return false;
    int64_t t = (int64_t)i->ct_mjd * 86400 + i->ct_hour * 3600 + i->ct_min * 60
              + (int64_t)i->ct_off_min * 60 + (int64_t)((uint32_t)(now_ms - i->ct_rx_ms) / 1000u);
    int64_t mjd = t / 86400; int64_t sod = t % 86400;
    if (sod < 0) { sod += 86400; mjd--; }

    int yp = (int)(((double)mjd - 15078.2) / 365.25);
    int mp = (int)(((double)mjd - 14956.1 - (int)(yp * 365.25)) / 30.6001);
    int day = (int)mjd - 14956 - (int)(yp * 365.25) - (int)(mp * 30.6001);
    int k = (mp == 14 || mp == 15) ? 1 : 0;
    o->year = 1900 + yp + k;
    o->mon  = mp - 1 - k * 12;
    o->day  = day;
    o->hour = (int)(sod / 3600);
    o->min  = (int)((sod / 60) % 60);
    o->sec  = (int)(sod % 60);
    o->off_min = i->ct_off_min;
    return true;
}
