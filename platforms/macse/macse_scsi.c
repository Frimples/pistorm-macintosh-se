/* SPDX-License-Identifier: MIT
 *
 * macse_scsi.c — NCR 5380 SCSI target emulation for the Macintosh SE.
 *
 * Written from the chip's documented register semantics and Apple's SCSI
 * Manager behaviour. MAME's macse_scsi_r/w and mac_scsi_helper were used as a
 * BEHAVIOURAL REFERENCE ONLY (it is BSD-3 and this tree is MIT) -- in
 * particular for the SE address decoding and for the pseudo-DMA constraint
 * described in macse_scsi.h. No MAME code was copied.
 *
 * See macse_scsi.h for the address decoding and the reason this route exists.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <limits.h>
#include "macse_scsi.h"

/* ---------------------------------------------------------------------- */
/*  Configuration                                                          */
/* ---------------------------------------------------------------------- */

#define SECTOR_SIZE     512
#define OUR_SCSI_ID     0        /* the volume the Mac will mount  */
#define MAX_SECTORS     0x20000  /* 64 MB ceiling, plenty for the SE */

/* Phases (the SCSI bus phase, reported via C/D, I/O, MSG) */
enum { PH_BUS_FREE = 0, PH_COMMAND, PH_DATA_IN, PH_DATA_OUT, PH_STATUS,
       PH_MSG_IN, PH_MSG_OUT };

/* Target selection state */
enum { TS_IDLE = 0, TS_SELECTED, TS_CMD, TS_DATA, TS_STATUS, TS_MSG, TS_DONE };

/* ---------------------------------------------------------------------- */
/*  State                                                                  */
/* ---------------------------------------------------------------------- */

static uint8_t  regs[1][8];           /* one NCR5380 register set */
static int ack_pending;
static int dma_dir; /* 0 stopped, 1 receive, 2 send */
static int irq_pending, dma_tail;
static uint8_t ack_data;
static uint8_t  out_data;             /* last byte written to reg 0       */
static int      sel_id;               /* ID latched when SEL was asserted */
static int      tstate   = TS_IDLE;
static int      phase    = PH_BUS_FREE;
static uint8_t  status_byte;

static uint8_t  cdb[16];
static int      cdb_len, cdb_pos;

static uint8_t *xfer_buf;             /* staging for the current phase    */
static int      xfer_len, xfer_pos;

static uint8_t  sense[18];

/* backing store */
static uint8_t *disk;
static uint32_t disk_sectors;
static char     disk_path[512];
static pthread_mutex_t disk_lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned long disk_generation;
static int disk_dirty;
static unsigned trace_left;
static void trace_access(char rw,uint32_t addr,uint8_t type,uint8_t data) {
    if (!trace_left) return;
    --trace_left;
    fprintf(stderr,"[SCSI-TRACE] %c %06x type=%u data=%02x phase=%d state=%d icr=%02x mode=%02x tcr=%02x cdb=%d/%d xfer=%d/%d\n",
        rw,addr,type,data,phase,tstate,regs[0][R_ICMD],regs[0][R_MODE],regs[0][R_TCMD],cdb_pos,cdb_len,xfer_pos,xfer_len);
}

/* ---------------------------------------------------------------------- */
/*  Helpers                                                                */
/* ---------------------------------------------------------------------- */

static void xfer_alloc(int len) {
    if (xfer_buf) { free(xfer_buf); xfer_buf = NULL; }
    xfer_len = xfer_pos = 0;
    if (len <= 0) return;
    xfer_buf = (uint8_t *)calloc(1, len);
    if (xfer_buf) xfer_len = len;
}

/* --- phase reporting --------------------------------------------------- */

static uint8_t curstat(void) {
    uint8_t v = 0;
    if (tstate != TS_IDLE || (regs[0][R_ICMD] & IC_BSY)) v |= ST_BSY;
    switch (phase) {
        case PH_COMMAND:  v |= ST_CD;            break;
        case PH_DATA_IN:  v |= ST_IO;            break;
        case PH_DATA_OUT:                        break;
        case PH_STATUS:   v |= ST_CD | ST_IO;    break;
        case PH_MSG_IN:   v |= ST_CD | ST_IO | ST_MSG; break;
        case PH_MSG_OUT:  v |= ST_CD | ST_MSG;   break;
        default: break;
    }
    if (regs[0][R_ICMD] & IC_SEL) v |= ST_SEL;
    if (regs[0][R_ICMD] & IC_RST) v |= ST_RST;
    if (phase != PH_BUS_FREE && !(regs[0][R_ICMD] & (IC_ACK|IC_RST))) v |= ST_REQ;
    return v;
}

static uint8_t busstat(void) {
    uint8_t v = 0;
    if ((regs[0][R_TCMD] & TC_PHASE) == ((curstat() >> 2) & 7)) v |= BAS_PHASE;
    if ((v & BAS_PHASE) && (curstat() & ST_REQ) &&
        (regs[0][R_MODE] & MODE_DMA) &&
        ((dma_dir == 1 && (curstat() & ST_IO)) ||
         (dma_dir == 2 && !(curstat() & ST_IO)))) v |= BAS_DRQ;
    if (regs[0][R_ICMD] & IC_ACK) v |= BAS_ACK;
    if (irq_pending) v |= BAS_IRQ;
    if (dma_tail) v |= BAS_DRQ;
    if (regs[0][R_ICMD] & IC_ATN) v |= BAS_ATN;
    return v;
}

/* --- command execution -------------------------------------------------- */

static void set_sense(uint8_t key, uint8_t asc, uint8_t ascq) {
    memset(sense, 0, sizeof(sense));
    sense[0] = 0x70;        /* current error, valid */
    sense[2] = key;
    sense[7] = 0x0A;        /* additional sense length */
    sense[12] = asc;
    sense[13] = ascq;
}

static void cmd_error(uint8_t key, uint8_t asc) {
    set_sense(key, asc, 0);
    status_byte = 0x02;     /* CHECK CONDITION */
    phase = PH_STATUS;
    tstate = TS_STATUS;
}

static void send_data(const uint8_t *src, int len) {
    xfer_alloc(len);
    if (xfer_buf && len > 0) memcpy(xfer_buf, src, len);
    if (len > 0 && !xfer_buf) { cmd_error(0x04,0x44); return; }
    phase = len ? PH_DATA_IN : PH_STATUS;
    tstate = len ? TS_DATA : TS_STATUS;
}

static void recv_data(int len) {
    xfer_alloc(len);
    if (len > 0 && !xfer_buf) { cmd_error(0x04,0x44); return; }
    phase = len ? PH_DATA_OUT : PH_STATUS;
    tstate = len ? TS_DATA : TS_STATUS;
}

static void execute_cdb(void) {
    uint8_t op = cdb[0];

    switch (op) {
    /* ---- TEST UNIT READY ---- */
    case 0x00:
        phase = PH_STATUS; tstate = TS_STATUS; status_byte = 0x00;
        break;

    /* ---- REQUEST SENSE ---- */
    case 0x03: {
        int n = cdb[4];
        if (n > (int)sizeof(sense)) n = sizeof(sense);
        send_data(sense, n);
        break;
    }

    /* ---- INQUIRY ---- */
    case 0x12: {
        int n = cdb[4]; if (n > 36) n = 36;
        uint8_t inq[36];
        memset(inq, 0, sizeof(inq));
        inq[0] = 0x00;                 /* direct-access device */
        inq[1] = 0x00;                 /* not removable */
        inq[2] = 0x02;                 /* SCSI-2 */
        inq[3] = 0x02;
        inq[4] = 31;                   /* additional length */
        memcpy(&inq[8],  "APPLE   ", 8);
        memcpy(&inq[16], "Hard Disk 20    ", 16);
        memcpy(&inq[32], "1.0 ", 4);
        send_data(inq, n);
        break;
    }

    /* ---- MODE SENSE(6) ---- */
    case 0x1A: {
        int n = cdb[4];
        uint8_t ms[40];
        uint32_t mb = disk_sectors / 2048;   /* "cylinders" for the Mac */
        memset(ms, 0, sizeof(ms));
        if ((cdb[2] & 0x3f) == 0x30) {
            /* Apple HD SC Setup probes vendor-specific page 30h and expects
             * this exact 20-byte Apple identity before offering the disk. */
            ms[0] = 33;                 /* bytes following this byte */
            ms[3] = 8;                  /* block descriptor length */
            ms[4] = (uint8_t)(mb >> 16);
            ms[5] = (uint8_t)(mb >> 8);
            ms[6] = (uint8_t)mb;
            ms[9] = (uint8_t)(SECTOR_SIZE >> 16);
            ms[10] = (uint8_t)(SECTOR_SIZE >> 8);
            ms[11] = (uint8_t)SECTOR_SIZE;
            ms[12] = 0x30; ms[13] = 20;
            memcpy(&ms[14], "APPLE COMPUTER, INC.", 20);
            if (n > 34) n = 34;
        } else {
            ms[0] = 12;                /* mode data length */
            ms[3] = 8;                 /* block descriptor length */
            ms[4] = 0;                 /* density */
            ms[5] = (uint8_t)((mb >> 16) & 0xFF);
            ms[6] = (uint8_t)((mb >> 8) & 0xFF);
            ms[7] = (uint8_t)(mb & 0xFF);
            ms[9] = 0; ms[10] = 0; ms[11] = 0;
            ms[12] = (uint8_t)((SECTOR_SIZE >> 16) & 0xFF);
            ms[13] = (uint8_t)((SECTOR_SIZE >> 8) & 0xFF);
            ms[14] = (uint8_t)(SECTOR_SIZE & 0xFF);
            if (n > 16) n = 16;
        }
        send_data(ms, n);
        break;
    }

    /* ---- FORMAT UNIT ----
     * The virtual medium has no defects or low-level geometry to format.
     * Clear its sectors and let Lido write the Macintosh filesystem structures. */
    case 0x04:
        pthread_mutex_lock(&disk_lock);
        if (disk && disk_sectors)
            memset(disk, 0, (size_t)disk_sectors * SECTOR_SIZE);
        disk_dirty = 1;
        ++disk_generation;
        pthread_mutex_unlock(&disk_lock);
        phase = PH_STATUS; tstate = TS_STATUS; status_byte = 0x00;
        break;

    /* ---- MODE SELECT(6)/(10) ----
     * Lido sends a mode parameter list before formatting.  The SE does not
     * need us to change any physical geometry; accept and consume the list
     * so the following FORMAT UNIT command can proceed. */
    case 0x15: case 0x55: {
        int n = op == 0x15 ? cdb[4] : (((int)cdb[7] << 8) | cdb[8]);
        recv_data(n);
        break;
    }

    /* ---- START STOP UNIT / PREVENT ALLOW ---- */
    case 0x1B: case 0x1E:
        phase = PH_STATUS; tstate = TS_STATUS; status_byte = 0x00;
        break;

    /* ---- READ CAPACITY(10) ---- */
    case 0x25: {
        uint8_t rc[8];
        uint32_t last = disk_sectors ? disk_sectors - 1 : 0;
        rc[0] = (uint8_t)(last >> 24); rc[1] = (uint8_t)(last >> 16);
        rc[2] = (uint8_t)(last >> 8);  rc[3] = (uint8_t)(last);
        rc[4] = 0; rc[5] = 0;
        rc[6] = (uint8_t)(SECTOR_SIZE >> 8); rc[7] = (uint8_t)(SECTOR_SIZE);
        send_data(rc, 8);
        break;
    }

    /* ---- READ(6) / READ(10) ---- */
    case 0x08: case 0x28: {
        uint32_t lba, cnt;
        if (op == 0x08) {
            lba = ((uint32_t)(cdb[1] & 31) << 16) | ((uint32_t)cdb[2] << 8) | cdb[3];
            cnt = cdb[4]; if (cnt == 0) cnt = 256;
        } else {
            lba = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) |
                  ((uint32_t)cdb[4] << 8)  | cdb[5];
            cnt = ((uint32_t)cdb[7] << 8) | cdb[8];
        }
        if (lba > disk_sectors || cnt > disk_sectors - lba) { cmd_error(0x05, 0x21); break; } /* LBA out of range */
        send_data(&disk[lba * SECTOR_SIZE], (int)(cnt * SECTOR_SIZE));
        break;
    }

    /* ---- WRITE(6) / WRITE(10) ---- */
    case 0x0A: case 0x2A: {
        uint32_t lba, cnt;
        if (op == 0x0A) {
            lba = ((uint32_t)(cdb[1] & 31) << 16) | ((uint32_t)cdb[2] << 8) | cdb[3];
            cnt = cdb[4]; if (cnt == 0) cnt = 256;
        } else {
            lba = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) |
                  ((uint32_t)cdb[4] << 8)  | cdb[5];
            cnt = ((uint32_t)cdb[7] << 8) | cdb[8];
        }
        if (lba > disk_sectors || cnt > disk_sectors - lba) { cmd_error(0x05, 0x21); break; }
        recv_data((int)(cnt * SECTOR_SIZE));
        /* remember the destination so the OUT phase can land the bytes */
        xfer_pos = 0;
        break;
    }

    default:
        cmd_error(0x05, 0x20);   /* illegal request / invalid command */
        break;
    }
}

static int cdb_length(uint8_t op) {
    switch (op >> 5) { /* SCSI group code, not opcode high nibble */
    case 0: return 6;
    case 1: case 2: return 10;
    case 4: return 16;
    case 5: return 12;
    default: return 6; /* reserved/vendor commands rejected by execute_cdb */
    }
}

/* ---------------------------------------------------------------------- */
/*  Byte-level transfer                                                    */
/* ---------------------------------------------------------------------- */

/* PIO reads only sample the bus. ACK rising latches the byte and drops
 * REQ; ACK falling advances the target and requests the next byte. */
static uint8_t peek_byte(void) {
    if (phase == PH_DATA_IN && xfer_pos < xfer_len) return xfer_buf[xfer_pos];
    if (phase == PH_STATUS) return status_byte;
    if (phase == PH_MSG_IN) return 0; /* COMMAND COMPLETE */
    return out_data;
}

static void advance_byte(uint8_t data) {
    int previous = phase;
    switch (phase) {
    case PH_COMMAND:
        if (cdb_pos < (int)sizeof(cdb)) cdb[cdb_pos++] = data;
        if (cdb_pos == 1) cdb_len = cdb_length(data);
        if (cdb_pos == cdb_len) execute_cdb();
        break;
    case PH_DATA_IN:
        if (++xfer_pos == xfer_len) { phase = PH_STATUS; tstate = TS_STATUS; }
        break;
    case PH_DATA_OUT:
        if (xfer_pos < xfer_len) xfer_buf[xfer_pos++] = data;
        if (xfer_pos == xfer_len) {
            uint32_t lba = cdb[0] == 0x0a ?
                ((uint32_t)(cdb[1]&31)<<16)|((uint32_t)cdb[2]<<8)|cdb[3] :
                ((uint32_t)cdb[2]<<24)|((uint32_t)cdb[3]<<16)|((uint32_t)cdb[4]<<8)|cdb[5];
            /* MODE SELECT parameter lists are controller configuration, not
             * disk sectors.  Never interpret their CDB bytes as an LBA. */
            if (cdb[0] != 0x15 && cdb[0] != 0x55 && xfer_len > 0) {
                pthread_mutex_lock(&disk_lock);
                memcpy(disk+(size_t)lba*SECTOR_SIZE,xfer_buf,xfer_len);
                disk_dirty = 1; ++disk_generation;
                pthread_mutex_unlock(&disk_lock);
            }
            phase = PH_STATUS; tstate = TS_STATUS;
        }
        break;
    case PH_STATUS: phase = PH_MSG_IN; tstate = TS_MSG; break;
    case PH_MSG_IN:
        phase = PH_BUS_FREE; tstate = TS_IDLE;
        cdb_pos = cdb_len = xfer_pos = xfer_len = 0;
        break;
    default: break;
    }
    if (phase != previous && (regs[0][R_MODE] & MODE_DMA)) {
        if (phase != PH_BUS_FREE && ((curstat() >> 2) & 7) != (regs[0][R_TCMD] & 7)) {
            irq_pending = 1;
            dma_tail = dma_dir == 2;
            dma_dir = 0;
        } else if (phase == PH_BUS_FREE) {
            dma_dir = dma_tail = 0;
            regs[0][R_MODE] &= ~MODE_DMA;
        }
    }
}

uint32_t macse_scsi_read(uint32_t addr, uint8_t type) {
    if (type == 2) { /* OP_TYPE_LONGWORD: big-endian ordered word cycles */
        uint32_t hi = macse_scsi_read(addr,1);
        return (hi << 16) | macse_scsi_read(addr+2,1);
    }
    int reg = ((addr - MACSE_SCSI_BASE) >> 4) & 7;
    uint8_t v = 0;
    switch (reg) {
    case R_CURDATA: v = peek_byte(); break;
    case R_INDATA:
        v = peek_byte();
        if (dma_dir == 1 && (busstat() & BAS_DRQ)) advance_byte(0);
        break;
    case R_ICMD: case R_MODE: case R_TCMD: v = regs[0][reg]; break;
    case R_CURSTAT: v = curstat(); break;
    case R_BUSSTAT: v = busstat(); break;
    case R_RESET: irq_pending = 0; break;
    default: break;
    }
    trace_access('R',addr,type,v);
    return type == 0 ? v : (uint32_t)v << 8;
}

int macse_scsi_write(uint32_t addr, uint32_t val, uint8_t type) {
    if (type == 2) {
        macse_scsi_write(addr,val >> 16,1);
        return macse_scsi_write(addr+2,val & 0xffff,1);
    }
    int reg = ((addr - MACSE_SCSI_BASE) >> 4) & 7;
    uint8_t data = (uint8_t)(type == 0 ? val : val >> 8);
    uint8_t old = regs[0][reg];
    uint8_t before = curstat();
    switch (reg) {
    case R_CURDATA:
        out_data = data;
        if ((addr & 0x200) && dma_tail) dma_tail = 0;
        else if ((addr & 0x200) && dma_dir == 2 && (busstat() & BAS_DRQ))
            advance_byte(data);
        break;
    case R_ICMD:
        regs[0][reg] = (data & 0x9f) | (old & (IC_AIP|IC_LA));
        if (data & IC_RST) {
            macse_scsi_reset(); regs[0][R_ICMD] = IC_RST; break;
        }
        if (phase == PH_BUS_FREE && tstate == TS_IDLE &&
            (data & (IC_SEL|IC_DBUS)) == (IC_SEL|IC_DBUS) &&
            (out_data & (1 << OUR_SCSI_ID))) {
            tstate = TS_SELECTED;
            cdb_pos = 0; cdb_len = 6; status_byte = 0;
            xfer_pos = xfer_len = 0;
        }
        if (tstate == TS_SELECTED && !(data & IC_SEL)) {
            tstate = TS_CMD; phase = PH_COMMAND;
        }
        if (!(old & IC_ACK) && (data & IC_ACK) && (before & ST_REQ)) {
            ack_pending = 1; ack_data = out_data;
        }
        if ((old & IC_ACK) && !(data & IC_ACK) && ack_pending) {
            ack_pending = 0; advance_byte(ack_data);
        }
        break;
    case R_MODE:
        regs[0][reg] = data;
        if (!(data & MODE_DMA)) dma_dir = dma_tail = 0;
        if (!(old & MODE_ARBITRATE) && (data & MODE_ARBITRATE) &&
            !(before & (ST_BSY|ST_SEL|ST_RST)))
            regs[0][R_ICMD] |= IC_AIP | IC_BSY;
        if (!(data & MODE_ARBITRATE)) regs[0][R_ICMD] &= ~(IC_AIP|IC_LA);
        break;
    case R_TCMD: regs[0][reg] = data & 15; break;
    case R_BUSSTAT:
        if (regs[0][R_MODE] & MODE_DMA) { dma_dir = 2; dma_tail = 0; }
        break;
    case R_INDATA: case R_RESET:
        if (regs[0][R_MODE] & MODE_DMA) { dma_dir = 1; dma_tail = 0; }
        break;
    default: break;
    }
    trace_access('W',addr,type,data);
    return 1;
}

/* ---------------------------------------------------------------------- */
/*  Lifecycle                                                              */
/* ---------------------------------------------------------------------- */

void macse_scsi_reset(void) {
    memset(regs, 0, sizeof(regs));
    out_data = 0; sel_id = 0; ack_pending = 0; dma_dir = 0;
    irq_pending = dma_tail = 0;
    tstate = TS_IDLE; phase = PH_BUS_FREE;
    cdb_pos = cdb_len = 0; status_byte = 0;
    xfer_pos = xfer_len = 0;
    memset(sense, 0, sizeof(sense));
    set_sense(0x00, 0x00, 0x00);
}

int macse_scsi_init(const char *image_path, uint32_t size_bytes) {
    struct stat st;
    if (!image_path || !image_path[0] || strlen(image_path) >= sizeof(disk_path)) return -1;
    /* Never round or clamp an existing file: a later save would destroy its tail. */
    if (stat(image_path,&st) != 0 || !S_ISREG(st.st_mode) || st.st_size <= 0 ||
        st.st_size > (int64_t)MAX_SECTORS*SECTOR_SIZE || st.st_size % SECTOR_SIZE ||
        (size_bytes && size_bytes != st.st_size)) {
        fprintf(stderr,"[MACSE-SCSI] rejected image: require existing whole-sector raw image <=64 MiB\n");
        return -1;
    }
    size_t len = (size_t)st.st_size;
    uint8_t *loaded = malloc(len);
    if (!loaded) return -2;
    FILE *f = fopen(image_path,"rb");
    if (!f) { free(loaded); return -1; }
    size_t got = fread(loaded,1,len,f);
    int err = ferror(f); fclose(f);
    if (got != len || err) { free(loaded); return -1; }
    char resolved[PATH_MAX];
    if (!realpath(image_path,resolved) || strlen(resolved) >= sizeof(disk_path)) {
        free(loaded); return -1;
    }
    pthread_mutex_lock(&disk_lock);
    free(disk); disk = loaded;
    disk_dirty = 0; ++disk_generation;
    disk_sectors = len/SECTOR_SIZE;
    memcpy(disk_path,resolved,strlen(resolved)+1);
    pthread_mutex_unlock(&disk_lock);
    macse_scsi_reset();
    const char *trace = getenv("MACSE_SCSI_TRACE");
    char *end = NULL;
    unsigned long limit = trace ? strtoul(trace,&end,10) : 0;
    trace_left = trace && end != trace && !*end ? (limit > 100000 ? 100000 : limit) : 0;
    printf("[MACSE-SCSI] loaded %zu bytes from %s (%u sectors)\n",len,disk_path,disk_sectors);
    return 0;
}

int macse_scsi_save(void) {
    /* Snapshot under the lock; slow SD I/O must not hold up CPU bus cycles.
     * Commit by rename only after a complete, flushed write. */
    char path[sizeof(disk_path)], tmp[sizeof(disk_path)+16];
    pthread_mutex_lock(&disk_lock);
    if (!disk || !disk_dirty) { pthread_mutex_unlock(&disk_lock); return 0; }
    size_t len = (size_t)disk_sectors*SECTOR_SIZE;
    uint8_t *snapshot = malloc(len);
    if (!snapshot) { pthread_mutex_unlock(&disk_lock); return 0; }
    memcpy(snapshot,disk,len); memcpy(path,disk_path,sizeof(path));
    unsigned long generation = disk_generation;
    pthread_mutex_unlock(&disk_lock);
    snprintf(tmp,sizeof(tmp),"%s.tmpXXXXXX",path);
    int fd = mkstemp(tmp), ok = fd >= 0;
    if (ok) {
        struct stat st;
        if (stat(path,&st) || fchmod(fd,st.st_mode & 0777)) ok = 0;
        size_t pos = 0;
        while (ok && pos < len) {
            ssize_t n = write(fd,snapshot+pos,len-pos);
            if (n <= 0) { ok = 0; break; }
            pos += (size_t)n;
        }
        if (ok && fsync(fd)) ok = 0;
        if (close(fd)) ok = 0;
        if (ok && rename(tmp,path)) ok = 0;
    }
    free(snapshot);
    if (!ok) { unlink(tmp); fprintf(stderr,"[MACSE-SCSI] save failed; original image retained\n"); return 0; }
    pthread_mutex_lock(&disk_lock);
    if (generation == disk_generation) disk_dirty = 0;
    pthread_mutex_unlock(&disk_lock);
    printf("[MACSE-SCSI] saved %zu sectors atomically\n",len/SECTOR_SIZE);
    return (int)(len/SECTOR_SIZE);
}
