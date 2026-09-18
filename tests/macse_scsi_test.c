/* CPU-facing NCR5380 regression. Addresses are Mac byte addresses, not
 * MAME's word offsets. Uses a private temporary image, never a user disk. */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "macse_scsi.h"
static int checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static uint8_t rd(int r) { return macse_scsi_read(0x580000u+16*r,0); }
static void wr(int r,uint8_t v) { macse_scsi_write(0x580001u+16*r,v,0); }
static void expect_phase(uint8_t p) {
    CHECK((rd(R_CURSTAT)&(ST_BSY|ST_REQ|ST_MSG|ST_CD|ST_IO))==(ST_BSY|ST_REQ|(p<<2)));
    wr(R_TCMD,p);
    CHECK(rd(R_BUSSTAT)&BAS_PHASE);
    wr(R_TCMD,p^1);
    CHECK(!(rd(R_BUSSTAT)&BAS_PHASE));
    wr(R_TCMD,p);
}
static void select0(void) {
    CHECK(!(rd(R_CURSTAT)&ST_BSY));
    wr(R_CURDATA,0x81);
    wr(R_ICMD,IC_SEL|IC_DBUS);
    CHECK(rd(R_CURSTAT)&ST_BSY);
    CHECK(!(rd(R_CURSTAT)&ST_REQ)); /* target waits for SEL release */
    wr(R_ICMD,0);
    expect_phase(2);
}
static void ack(uint8_t flags) {
    wr(R_ICMD,flags|IC_ACK);
    CHECK(!(rd(R_CURSTAT)&ST_REQ));
    CHECK(rd(R_BUSSTAT)&BAS_ACK);
    wr(R_ICMD,flags|IC_ACK); /* held ACK must not double-consume */
    CHECK(!(rd(R_CURSTAT)&ST_REQ));
    wr(R_ICMD,flags);
}
static void command(const uint8_t *c,int n) {
    for(int i=0;i<n;i++) {
        expect_phase(2);
        wr(R_CURDATA,c[i]);
        expect_phase(2); /* output latch write is NOT a handshake */
        ack(IC_DBUS);
    }
    wr(R_ICMD,0);
}
static uint8_t pio_in(void) {
    uint8_t b=rd(R_CURDATA);
    CHECK(rd(R_CURDATA)==b);
    CHECK(rd(R_INDATA)==b); /* ordinary reads must not advance */
    ack(0);
    return b;
}
static void finish(uint8_t s) {
    wr(R_MODE,0);
    expect_phase(3); CHECK(pio_in()==s);
    expect_phase(7); CHECK(pio_in()==0);
    CHECK(!(rd(R_CURSTAT)&(ST_BSY|ST_REQ)));
}
static void dma_start(int input,uint8_t p) {
    expect_phase(p);
    wr(R_MODE,MODE_DMA);
    wr(input ? R_RESET : R_BUSSTAT,0); /* start initiator receive / send */
    CHECK(rd(R_BUSSTAT)&BAS_DRQ);
}
static uint8_t dma_in(int i) {
    CHECK(rd(R_BUSSTAT)&BAS_DRQ);
    /* MOVEP.L walks A1/A2 while addressing the same data register. */
    return macse_scsi_read(0x580260u+2*(i&3),0);
}
int main(void) {
    char image[]="/tmp/macse-scsi-XXXXXX";
    int fd=mkstemp(image); CHECK(fd>=0);
    uint8_t original[4096];
    for(int i=0;i<4096;i++) original[i]=(uint8_t)(i*7+(i/512));
    CHECK(write(fd,original,sizeof(original))==(ssize_t)sizeof(original)); close(fd);
    CHECK(macse_scsi_init(image,0)==0);
    wr(R_MODE,0x20);
    CHECK(rd(R_MODE)==0x20);
    CHECK(macse_scsi_read(0x580020,1)==0x2000);
    macse_scsi_write(0x580020,0x1000,1);
    CHECK(rd(R_MODE)==0x10);
    CHECK(macse_scsi_read(0x580220,0)==0x10); /* A9 is DMA, not bank */
    CHECK(macse_scsi_read(0x580120,0)==0x10); /* mirrored register */
    CHECK(macse_scsi_read(0x580020,2)==0x10001000); /* two word bus cycles */
    macse_scsi_write(0x580020,0x20003000,2);
    CHECK(rd(R_MODE)==0x30);
    macse_scsi_reset();
    wr(R_CURDATA,0x80);
    wr(R_MODE,MODE_ARBITRATE);
    CHECK((rd(R_ICMD)&(IC_AIP|IC_LA))==IC_AIP);
    CHECK(rd(R_CURSTAT)&ST_BSY);
    CHECK(rd(R_CURDATA)==0x80);
    wr(R_MODE,0);
    CHECK(!(rd(R_ICMD)&IC_AIP));
    macse_scsi_reset();
    const uint8_t tur[6]={0};
    for(int i=0;i<3;i++) { select0(); command(tur,6); finish(0); }
    uint8_t inq[36];
    const uint8_t inquiry[6]={0x12,0,0,0,36,0};
    select0(); command(inquiry,6); dma_start(1,1);
    for(int i=0;i<36;i++) inq[i]=dma_in(i);
    CHECK(!memcmp(inq+8,"APPLE   ",8));
    CHECK(!memcmp(inq+16,"Hard Disk 20    ",16));
    CHECK(!(rd(R_BUSSTAT)&BAS_DRQ)); /* no spilling into status on phase mismatch */
    finish(0);
    select0(); dma_start(0,2);
    for(int i=0;i<6;i++) macse_scsi_write(0x580201u+2*(i&3),inquiry[i],0);
    dma_start(1,1);
    for(int i=0;i<36;i++) CHECK(dma_in(i)==inq[i]);
    finish(0);
    const uint8_t capacity[10]={0x25,0,0,0,0,0,0,0,0,0};
    select0(); command(capacity,10); dma_start(1,1);
    const uint8_t capwant[8]={0,0,0,7,0,0,2,0};
    for(int i=0;i<8;i++) CHECK(dma_in(i)==capwant[i]);
    finish(0);
    const uint8_t apple_page[6]={0x1a,0,0x30,0,34,0};
    uint8_t page[34];
    select0(); command(apple_page,6); dma_start(1,1);
    for(int i=0;i<34;i++) page[i]=dma_in(i);
    CHECK(page[0]==33 && page[12]==0x30 && page[13]==20);
    CHECK(!memcmp(page+14,"APPLE COMPUTER, INC.",20));
    finish(0);
    /* MODE SELECT(6): Lido sends this before formatting.  It must be
     * accepted as a parameter-list transfer and must not write sector 0. */
    const uint8_t mode_select[6]={0x15,0x10,0,0,12,0};
    uint8_t mode_params[12]={0,0,0,8,0,0,0,8,0,0,2,0};
    select0(); command(mode_select,6); dma_start(0,0);
    for(int i=0;i<12;i++) macse_scsi_write(0x580201u,mode_params[i],0);
    finish(0);
    const uint8_t format_unit[6]={0x04,0,0,0,0,0};
    select0(); command(format_unit,6); finish(0);
    const uint8_t write10[10]={0x2a,0,0,0,0,3,0,0,1,0};
    const uint8_t read10[10]={0x28,0,0,0,0,3,0,0,1,0};
    select0(); command(write10,10); dma_start(0,0);
    for(int i=0;i<512;i++) macse_scsi_write(0x580201u+2*(i&3),(uint8_t)(i^0xa5),0);
    CHECK(rd(R_BUSSTAT)&BAS_IRQ); /* DMA phase mismatch latches IRQ */
    CHECK(rd(R_BUSSTAT)&BAS_DRQ); /* final send DACK still requested */
    macse_scsi_write(0x580201,0xff,0); /* drain request, MUST NOT eat status */
    CHECK(!(rd(R_BUSSTAT)&BAS_DRQ));
    rd(R_RESET); CHECK(!(rd(R_BUSSTAT)&BAS_IRQ));
    finish(0);
    select0(); command(read10,10); dma_start(1,1);
    for(int i=0;i<512;i++) CHECK(dma_in(i)==(uint8_t)(i^0xa5));
    finish(0);
    const uint8_t write6[6]={0x0a,0,0,3,1,0};
    const uint8_t read6[6]={0x08,0,0,3,1,0};
    select0(); command(write6,6); expect_phase(0);
    for(int i=0;i<512;i++) { wr(R_CURDATA,(uint8_t)(i^0xa5)); ack(IC_DBUS); }
    wr(R_ICMD,0); finish(0);
    select0(); command(read6,6); expect_phase(1);
    for(int i=0;i<512;i++) CHECK(pio_in()==(uint8_t)(i^0xa5));
    finish(0);
    const uint8_t zero_inquiry[6]={0x12,0,0,0,0,0};
    const uint8_t zero_sense[6]={3,0,0,0,0,0};
    const uint8_t zero_mode[6]={0x1a,0,0,0,0,0};
    const uint8_t zero_read[10]={0x28,0,0,0,0,0,0,0,0,0};
    const uint8_t zero_write[10]={0x2a,0,0,0,0,0,0,0,0,0};
    select0(); command(zero_inquiry,6); finish(0);
    select0(); command(zero_sense,6); finish(0);
    select0(); command(zero_mode,6); finish(0);
    select0(); command(zero_read,10); finish(0);
    select0(); command(zero_write,10); finish(0);
    const uint8_t overflow_write[10]={0x2a,0,0xff,0xff,0xff,0xff,0,0,1,0};
    select0(); command(overflow_write,10); finish(2);
    const uint8_t overflow_read[10]={0x28,0,0xff,0xff,0xff,0xff,0,0,1,0};
    select0(); command(overflow_read,10); finish(2);
    /* Incomplete write must never reach the disk, including after reset. */
    select0(); command(write10,10); dma_start(0,0);
    for(int i=0;i<511;i++) macse_scsi_write(0x580201,0xee,0);
    wr(R_ICMD,IC_RST); CHECK(rd(R_CURSTAT)&ST_RST); wr(R_ICMD,0);
    select0(); command(read10,10); dma_start(1,1);
    for(int i=0;i<512;i++) CHECK(dma_in(i)==(uint8_t)(i^0xa5));
    finish(0);
    char backup[128]; snprintf(backup,sizeof(backup),"%s.backup",image);
    CHECK(link(image,backup)==0); /* old inode must survive atomic replacement */
    CHECK(macse_scsi_save()==8);
    FILE *oldfile=fopen(backup,"rb"); CHECK(oldfile!=NULL);
    uint8_t oldbytes[4096]; CHECK(fread(oldbytes,1,sizeof(oldbytes),oldfile)==sizeof(oldbytes)); fclose(oldfile);
    CHECK(!memcmp(oldbytes,original,sizeof(original))); unlink(backup);
    FILE *f=fopen(image,"rb"); CHECK(f!=NULL);
    uint8_t saved[4096]; CHECK(fread(saved,1,sizeof(saved),f)==sizeof(saved)); fclose(f);
    for(int i=0;i<4096;i++) CHECK(saved[i]==(i>=1536 && i<2048 ? (uint8_t)((i-1536)^0xa5) : 0));
    /* Refuse malformed/oversized images instead of silently truncating on save. */
    char bad[]="/tmp/macse-scsi-bad-XXXXXX";
    fd=mkstemp(bad); CHECK(fd>=0);
    CHECK(ftruncate(fd,513)==0);
    CHECK(macse_scsi_init(bad,0)!=0);
    CHECK(ftruncate(fd,65*1024*1024)==0);
    CHECK(macse_scsi_init(bad,0)!=0);
    close(fd); unlink(bad);
    unlink(image);
    printf("PASS %d CPU-facing checks\n",checks);
    return 0;
}
