/*
 * rds_group.h – RDS-Blocksynchronisation, CRC, Gruppendekodierung
 *
 * Reine C-Bibliothek ohne ESP-IDF-Abhängigkeit.
 * Eingang: Bitstrom (nach Differenzdekodierung). Ausgang: rds_info_t
 * (PI, PS, PTY, RadioText, RT+, Uhrzeit) sowie optional jede Rohgruppe.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RDS_PS_LEN   8
#define RDS_RT_LEN   64

/* Eine empfangene Gruppe (4 Blöcke à 16 Datenbit) */
typedef struct {
    uint16_t blk[4];
    uint8_t  ok;         /* Bit i = Block i hat gültige Prüfsumme              */
    uint8_t  corrected;  /* Bit i = Block i wurde per Bündelfehler-Korrektur repariert */
    bool     c_prime;    /* Block 2 trug Offset C' (Gruppentyp B)              */
} rds_group_t;

typedef struct { uint8_t type, start, len; } rds_rtp_tag_t;

/* Gesamtzustand – wird vom Dekoder fortlaufend aktualisiert */
typedef struct {
    bool     synced;
    bool     pi_valid;   uint16_t pi;
    uint8_t  pty;        bool tp, ta, ms;   /* ms: 1 = Musik, 0 = Sprache       */

    uint8_t  ps[RDS_PS_LEN];  uint8_t ps_seen;      /* Bitmaske der 4 Segmente  */

    uint8_t  rt[RDS_RT_LEN];  uint64_t rt_seen;     /* Bitmaske je Zeichen      */
    uint8_t  rt_ab;           bool rt_valid;

    uint8_t  rtp_gtype;                              /* 0xFF = RT+ unbekannt     */
    bool     rtp_valid, rtp_running;
    rds_rtp_tag_t rtp_tag[2];

    bool     ct_valid;
    int32_t  ct_mjd;  uint8_t ct_hour, ct_min;       /* UTC                      */
    int16_t  ct_off_min;                             /* lokaler Offset in Minuten*/
    uint32_t ct_rx_ms;                               /* Empfangszeitpunkt        */

    /* Statistik */
    uint32_t groups_ok, blocks_ok, blocks_bad, blocks_corrected, sync_losses;
} rds_info_t;

typedef struct { int year, mon, day, hour, min, sec; int off_min; } rds_datetime_t;

typedef void (*rds_group_cb_t)(const rds_group_t *g, void *ctx);

typedef struct {
    rds_info_t     info;
    rds_group_cb_t on_group;
    void          *ctx;

    /* interne Synchronisationszustände */
    uint32_t reg;            /* Schieberegister der letzten 26 Bit            */
    bool     synced;
    int      last_off;       /* zuletzt gefundenes Offset-Wort (-1 = keins)   */
    int      since_valid;    /* Bits seit diesem Fund                         */
    int      blk_idx;        /* erwarteter Block 0..3                         */
    int      blk_bits;       /* Bits im laufenden Block                       */
    rds_group_t cur;
    int      win_n, win_bad; /* Fehlerfenster für Sync-Verlust                */

    /* Zwischenspeicher zur Bestätigung korrigierter Segmente */
    uint8_t  ps_pend[RDS_PS_LEN];  uint8_t ps_pend_mask;
    uint8_t  rt_pend[RDS_RT_LEN];  uint64_t rt_pend_mask;
} rds_decoder_t;

void rds_decoder_init(rds_decoder_t *d, rds_group_cb_t on_group, void *ctx);
void rds_decoder_push_bit(rds_decoder_t *d, uint8_t bit, uint32_t now_ms);

/* --- Hilfsfunktionen für die Ausgabe --- */

/* RDS-Zeichensatz (EBU Latin) -> UTF-8, gibt Länge zurück */
size_t rds_text_utf8(const uint8_t *src, size_t len, char *dst, size_t dst_sz);

/* Länge des RadioTexts (bis CR bzw. ohne Leerzeichen am Ende) */
size_t rds_rt_length(const rds_info_t *i);

/* RT+ Tag (z. B. 1 = Titel, 4 = Interpret) als UTF-8 holen; false wenn nicht verfügbar */
bool rds_rtp_text(const rds_info_t *i, uint8_t content_type, char *out, size_t out_sz);

/* Lokale Uhrzeit aus letzter CT-Gruppe + verstrichener Zeit */
bool rds_local_time(const rds_info_t *i, uint32_t now_ms, rds_datetime_t *out);

#ifdef __cplusplus
}
#endif
