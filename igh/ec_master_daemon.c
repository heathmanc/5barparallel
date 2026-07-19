/* IgH EtherCAT RT master daemon for the 5-bar robot.
 *
 * Owns the DC cyclic loop (the config proven by igh_test) for 1-2 ANCTL AS715N
 * drives and exposes each drive's process image + a CSP setpoint buffer in POSIX
 * shared memory (/bcr_ethercat). The Python IgHMaster maps that memory and drives
 * the CiA 402 state machine / streams motion; this daemon is the RT transport.
 *
 * Usage:  sudo ./ec_master_daemon --drives 1 [--cycle-ns 2000000]
 * Stop:   set shm.stop = 1 (IgHMaster does this) or SIGINT.
 *
 * Shared-memory layout is mirrored byte-for-byte by igh_master.py (packed, LE).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sched.h>

#include "ecrt.h"

#define VENDOR       0x00400000u
#define PRODUCT      0x00000715u
#define ASSIGN_ACT   0x0300
#define SYNC0_SHIFT  0
#define MAX_DRIVES   2
#define CSP_MAX      65536
#define SHM_NAME     "/bcr_ethercat"
#define SHM_MAGIC    0x42435231u   /* 'BCR1' */
#define SHM_ABI      1u

#define NSEC_PER_SEC 1000000000L
#define TS2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#pragma pack(push, 1)
typedef struct {
    /* outputs: Python -> daemon */
    uint16_t controlword;
    int8_t   mode;
    uint8_t  _p0;
    int32_t  target_position;
    uint32_t digital_outputs;
    /* inputs: daemon -> Python */
    uint16_t statusword;
    int8_t   mode_display;
    uint8_t  _p1;
    int32_t  actual_position;
    int32_t  following_error;
    uint16_t error_code;
    int16_t  torque_actual;
    uint32_t digital_inputs;
} drive_shm_t;

typedef struct {
    uint32_t magic, abi, num_drives, cycle_dt_ns;
    uint64_t cycle_count;
    uint32_t wkc_bad, op, stop;
    uint32_t csp_start, csp_running, csp_len, csp_index;
    drive_shm_t drive[MAX_DRIVES];
    int32_t  csp[MAX_DRIVES][CSP_MAX];
} shm_layout_t;
#pragma pack(pop)

/* RxPDO 0x1701 / TxPDO 0x1B01 (from ec_inspect.py) */
static ec_pdo_entry_info_t rx_entries[] = {
    {0x6040,0,16},{0x607A,0,32},{0x60B8,0,16},{0x60FE,1,32}};
static ec_pdo_entry_info_t tx_entries[] = {
    {0x603F,0,16},{0x6041,0,16},{0x6064,0,32},{0x6077,0,16},{0x60F4,0,32},
    {0x60B9,0,16},{0x60BA,0,32},{0x60BC,0,32},{0x60FD,0,32}};
static ec_pdo_info_t rx_pdos[] = {{0x1701, 4, rx_entries}};
static ec_pdo_info_t tx_pdos[] = {{0x1B01, 9, tx_entries}};
static ec_sync_info_t syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, rx_pdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, tx_pdos, EC_WD_DISABLE},
    {0xff}};

struct off {                       /* per-drive process-image byte offsets */
    unsigned ctrl, target, tp, dout;
    unsigned err, status, pos, torq, ferr, tps, tp1, tp2, din;
};

static volatile sig_atomic_t running = 1;
static void on_sig(int s){ (void)s; running = 0; }

int main(int argc, char **argv)
{
    int num_drives = 1;
    long cycle_ns = 2000000L;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--drives") && i+1 < argc) num_drives = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--cycle-ns") && i+1 < argc) cycle_ns = atol(argv[++i]);
    }
    if (num_drives < 1) num_drives = 1;
    if (num_drives > MAX_DRIVES) num_drives = MAX_DRIVES;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    /* shared memory */
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0660);
    if (fd < 0) { perror("shm_open"); return 1; }
    if (ftruncate(fd, sizeof(shm_layout_t))) { perror("ftruncate"); return 1; }
    shm_layout_t *shm = mmap(NULL, sizeof(shm_layout_t), PROT_READ|PROT_WRITE,
                             MAP_SHARED, fd, 0);
    if (shm == MAP_FAILED) { perror("mmap"); return 1; }
    memset(shm, 0, sizeof(*shm));
    shm->magic = SHM_MAGIC; shm->abi = SHM_ABI;
    shm->num_drives = num_drives; shm->cycle_dt_ns = (uint32_t)cycle_ns;

    /* IgH master + slaves */
    ec_master_t *master = ecrt_request_master(0);
    if (!master) { fprintf(stderr, "request_master failed\n"); return 1; }
    ec_domain_t *domain = ecrt_master_create_domain(master);

    struct off off[MAX_DRIVES];
    static ec_pdo_entry_reg_t regs[MAX_DRIVES*13 + 1];
    int r = 0;
    for (int d = 0; d < num_drives; d++) {
        ec_slave_config_t *sc = ecrt_master_slave_config(master, 0, d, VENDOR, PRODUCT);
        if (!sc) { fprintf(stderr, "slave_config[%d] failed\n", d); return 1; }
        if (ecrt_slave_config_pdos(sc, EC_END, syncs)) { fprintf(stderr, "pdos[%d]\n", d); return 1; }
        ecrt_slave_config_sdo8(sc, 0x6060, 0, 8);                  /* CSP */
        ecrt_slave_config_dc(sc, ASSIGN_ACT, cycle_ns, SYNC0_SHIFT, 0, 0);
        ec_pdo_entry_reg_t e[] = {
            {0,d,VENDOR,PRODUCT,0x6040,0,&off[d].ctrl},
            {0,d,VENDOR,PRODUCT,0x607A,0,&off[d].target},
            {0,d,VENDOR,PRODUCT,0x60B8,0,&off[d].tp},
            {0,d,VENDOR,PRODUCT,0x60FE,1,&off[d].dout},
            {0,d,VENDOR,PRODUCT,0x603F,0,&off[d].err},
            {0,d,VENDOR,PRODUCT,0x6041,0,&off[d].status},
            {0,d,VENDOR,PRODUCT,0x6064,0,&off[d].pos},
            {0,d,VENDOR,PRODUCT,0x6077,0,&off[d].torq},
            {0,d,VENDOR,PRODUCT,0x60F4,0,&off[d].ferr},
            {0,d,VENDOR,PRODUCT,0x60B9,0,&off[d].tps},
            {0,d,VENDOR,PRODUCT,0x60BA,0,&off[d].tp1},
            {0,d,VENDOR,PRODUCT,0x60BC,0,&off[d].tp2},
            {0,d,VENDOR,PRODUCT,0x60FD,0,&off[d].din},
        };
        for (unsigned k = 0; k < sizeof(e)/sizeof(e[0]); k++) regs[r++] = e[k];
    }
    memset(&regs[r], 0, sizeof(regs[r]));   /* terminator */

    if (ecrt_domain_reg_pdo_entry_list(domain, regs)) { fprintf(stderr,"reg_pdo\n"); return 1; }
    if (ecrt_master_activate(master)) { fprintf(stderr,"activate\n"); return 1; }
    uint8_t *pd = ecrt_domain_data(domain);

    mlockall(MCL_CURRENT | MCL_FUTURE);
    struct sched_param sp = { .sched_priority = 80 };
    sched_setscheduler(0, SCHED_FIFO, &sp);

    struct timespec wk; clock_gettime(CLOCK_MONOTONIC, &wk);
    fprintf(stderr, "ec_master_daemon: %d drive(s), %ld ns cycle, running\n",
            num_drives, cycle_ns);

    while (running && !shm->stop) {
        wk.tv_nsec += cycle_ns;
        while (wk.tv_nsec >= NSEC_PER_SEC) { wk.tv_nsec -= NSEC_PER_SEC; wk.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wk, NULL);

        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        /* inputs: PDO -> shm */
        for (int d = 0; d < num_drives; d++) {
            drive_shm_t *v = &shm->drive[d];
            v->statusword      = EC_READ_U16(pd + off[d].status);
            v->actual_position = EC_READ_S32(pd + off[d].pos);
            v->following_error = EC_READ_S32(pd + off[d].ferr);
            v->error_code      = EC_READ_U16(pd + off[d].err);
            v->torque_actual   = EC_READ_S16(pd + off[d].torq);
            v->digital_inputs  = EC_READ_U32(pd + off[d].din);
            v->mode_display    = v->mode;
        }

        /* CSP setpoint streaming (Python arms via csp_start) */
        if (shm->csp_start) { shm->csp_index = 0; shm->csp_running = 1; shm->csp_start = 0; }
        int streaming = shm->csp_running && shm->csp_index < shm->csp_len;

        /* outputs: shm -> PDO */
        for (int d = 0; d < num_drives; d++) {
            drive_shm_t *v = &shm->drive[d];
            int op_enabled = (v->statusword & 0x0004);   /* CiA402 Operation Enabled */
            int32_t target;
            if (streaming)          target = shm->csp[d][shm->csp_index];
            else if (!op_enabled)   target = v->actual_position;  /* track -> no enable jump */
            else                    target = v->target_position;  /* hold at commanded */
            v->target_position = target;              /* reflect what was applied */
            EC_WRITE_U16(pd + off[d].ctrl,  v->controlword);
            EC_WRITE_S32(pd + off[d].target, target);
            EC_WRITE_U16(pd + off[d].tp,    0);
            EC_WRITE_U32(pd + off[d].dout,  v->digital_outputs);
        }
        if (streaming) { shm->csp_index++; if (shm->csp_index >= shm->csp_len) shm->csp_running = 0; }

        /* distributed clocks */
        ecrt_master_application_time(master, TS2NS(wk));
        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        ec_master_state_t ms; ecrt_master_state(master, &ms);
        shm->op = (ms.al_states & 0x08) ? 1 : 0;
        shm->cycle_count++;

        /* ~1 Hz heartbeat so the daemon can be verified standalone */
        if (shm->cycle_count % (uint64_t)(NSEC_PER_SEC / cycle_ns) == 0)
            fprintf(stderr, "  op=%u  d0 status=0x%04X err=0x%04X pos=%d  wkc_bad=%u\n",
                    shm->op, shm->drive[0].statusword, shm->drive[0].error_code,
                    shm->drive[0].actual_position, shm->wkc_bad);
    }

    /* Safety: command the drives disabled (controlword=0 -> torque off) for a
     * short burst, with DC maintained, before tearing down — so a stop via the
     * flag, SIGTERM, or pkill leaves the drives disabled cleanly rather than
     * relying on the drive's comms-loss watchdog. */
    fprintf(stderr, "ec_master_daemon: stopping (commanding disable)\n");
    for (int i = 0; i < 50; i++) {
        wk.tv_nsec += cycle_ns;
        while (wk.tv_nsec >= NSEC_PER_SEC) { wk.tv_nsec -= NSEC_PER_SEC; wk.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wk, NULL);
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        for (int d = 0; d < num_drives; d++) {
            EC_WRITE_U16(pd + off[d].ctrl, 0x0000);       /* disable voltage */
            EC_WRITE_S32(pd + off[d].target, shm->drive[d].actual_position);
        }
        ecrt_master_application_time(master, TS2NS(wk));
        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain);
        ecrt_master_send(master);
    }
    ecrt_master_deactivate(master);
    ecrt_release_master(master);
    munmap(shm, sizeof(*shm));
    shm_unlink(SHM_NAME);
    return 0;
}
