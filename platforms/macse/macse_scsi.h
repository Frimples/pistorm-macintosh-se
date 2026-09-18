/* SPDX-License-Identifier: MIT
 *
 * macse_scsi.h — NCR 5380 SCSI emulation for the Macintosh SE under PiStorm.
 *
 * WHY THIS EXISTS
 * ---------------
 * The SE has a complete, working disk stack the PiStorm currently ignores:
 * an NCR 5380 SCSI controller at $580000. Because macse-platform.c sets all
 * I/O handlers to NULL, SCSI accesses fall through to the real bus, hit the
 * real 5380, and find nothing -- which is why the fork's README says
 * "The SCSI system is not working and no hard drives are detected."
 *
 * Emulating the chip instead means the Mac's OWN SCSI driver probes, mounts
 * and boots the volume. No custom DRVR, no AddDrive()/INIT, no drive-queue
 * surgery, no fake HFS -- which is exactly the mounting problem that blocks
 * the custom-driver route.
 *
 * SE ADDRESS DECODING  (from MAME mac128.cpp :: macse_map / macse_scsi_r/w)
 * -------------------------------------------------------------------------
 *   window    $580000 .. $5FFFFF (512 KiB, heavily mirrored)
 *   register  reg = (byte_offset >> 4) & 7 (16-byte stride)
 *   DMA       byte address A9 ($200); NOT a second register bank
 *   data      byte accesses return/accept low 8 bits of the API value;
 *             word accesses carry data in bits 15..8. Longword accesses
 *             split into two ordered word bus cycles.
 *   aliases   read $580000+16*r, write $580001+16*r; MOVEP uses +0,+2,+4,+6.
 * MAME u16 handlers receive WORD offsets, not CPU byte addresses.
 *
 * PSEUDO-DMA
 * ----------
 * The Mac Toolbox implements SCSIRead/SCSIWrite by polling DRQ, and
 * SCSIRBlind/SCSIWBlind by NOT polling -- the latter rely on DRQ being wired
 * to DTACK so the hardware inserts wait states. On the SE these blind loops
 * are unrolled MOVEP.L sequences moving 4 bytes at a time.
 *
 * Unlike MAME we do not need FIFOs or CPU halting: our "SCSI device" is a
 * function call in the same process, so a byte is available the instant the
 * CPU asks. The one real constraint is that a blind MOVEP.L consumes 4 bytes
 * before it looks at anything, so the target must never run a data phase
 * short of a 4-byte boundary mid-instruction. Every data phase Apple's code
 * performs is a multiple of 4 (36-byte INQUIRY, 8-byte READ CAPACITY,
 * 512-byte sectors), which is not a coincidence.
 */

#ifndef _MACSE_SCSI_H
#define _MACSE_SCSI_H

#include <stdint.h>

/* --- SE SCSI window ---------------------------------------------------- */
#define MACSE_SCSI_BASE   0x580000u
#define MACSE_SCSI_HIGH   0x600000u   /* exclusive */

/* --- NCR 5380 register indices (offset & 7) ---------------------------- */
#define R_CURDATA   0   /* R: current SCSI data      W: output data      */
#define R_ICMD      1   /*    initiator command                          */
#define R_MODE      2   /*    mode                                       */
#define R_TCMD      3   /*    target command                             */
#define R_CURSTAT   4   /* R: current SCSI bus status W: select enable   */
#define R_BUSSTAT   5   /* R: bus and status         W: start DMA send   */
#define R_INDATA    6   /* R: input data W: start DMA target receive     */
#define R_RESET     7   /* R: clear IRQ W: start DMA initiator receive   */

/* register 1 — initiator command (write) / (read) */
#define IC_RST      0x80   /* assert RST */
#define IC_AIP      0x40   /* (read) arbitration in progress */
#define IC_TEST     0x40   /* (write) test mode */
#define IC_LA       0x20   /* (read) lost arbitration */
#define IC_ACK      0x10   /* assert ACK */
#define IC_BSY      0x08   /* assert BSY */
#define IC_SEL      0x04   /* assert SEL */
#define IC_ATN      0x02   /* assert ATN */
#define IC_DBUS     0x01   /* assert data bus */

/* register 2 — mode */
#define MODE_BLOCKDMA   0x80
#define MODE_TARGET     0x40
#define MODE_PARITYCHK  0x20
#define MODE_PARITYIRQ  0x10
#define MODE_EOPIRQ     0x08
#define MODE_BSYIRQ     0x04
#define MODE_DMA        0x02   /* pseudo-DMA / DMA mode enable */
#define MODE_ARBITRATE  0x01

/* register 3 — target command (read) */
#define TC_REQ      0x08
#define TC_MSG      0x04
#define TC_CD       0x02
#define TC_IO       0x01
#define TC_PHASE    0x07

/* register 4 — current SCSI bus status */
#define ST_RST      0x80
#define ST_BSY      0x40
#define ST_REQ      0x20
#define ST_MSG      0x10
#define ST_CD       0x08
#define ST_IO       0x04
#define ST_SEL      0x02
#define ST_DBP      0x01

/* register 5 — bus and status */
#define BAS_ENDDMA  0x80
#define BAS_DRQ     0x40
#define BAS_PARITY  0x20
#define BAS_IRQ     0x10
#define BAS_PHASE   0x08
#define BAS_BSYERR  0x04
#define BAS_ATN     0x02
#define BAS_ACK     0x01

/* injected status codes handed back to the guest on fatal transfer errors
 * (the caller turns these into a bus error, which the Mac's SCSI Manager
 * catches through its own exception handler). */
#define MACSE_SCSI_ERR_READ   1
#define MACSE_SCSI_ERR_WRITE  0

/* ---------------------------------------------------------------------- */
/*  API                                                                    */
/* ---------------------------------------------------------------------- */

/* Attach an existing whole-sector raw image <=64 MiB. size_bytes must be
 * zero (auto) or match exactly. Returns 0 on success, non-zero on failure.
 * Never truncates or rounds an image; failure retains any prior image. */
int  macse_scsi_init(const char *image_path, uint32_t size_bytes);

/* Atomically save a dirty snapshot. Returns sectors written, or zero if
 * clean or failed. Failed saves retain dirty state. */
int  macse_scsi_save(void);

void macse_scsi_reset(void);

/* True if the address falls in the SE SCSI window. */
static inline int macse_scsi_owns(uint32_t addr) {
    return addr >= MACSE_SCSI_BASE && addr < MACSE_SCSI_HIGH;
}

/* Bus-cycle entry points: type 0=byte, 1=word, 2=longword.
 * MACSE_SCSI_TRACE=N logs at most N accesses to stderr (cap 100000). */
uint32_t macse_scsi_read(uint32_t addr, uint8_t type);
int      macse_scsi_write(uint32_t addr, uint32_t val, uint8_t type);

#endif /* _MACSE_SCSI_H */
