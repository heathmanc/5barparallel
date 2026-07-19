/* IgH EtherCAT master DC go/no-go test for the ANCTL AS715N (StepperOnline A6-EC).
 *
 * pysoem cannot generate a SYNC0 this DC-only drive accepts (it faults Er74.1
 * "No sync signal" / 0x8700). This program does the same job with the IgH
 * EtherLab master, whose DC implementation is complete. If the drive reaches OP
 * and holds err=0x0000, IgH is the path; if it also shows 0x8700, it isn't.
 *
 * No motion: the controlword is held at 0 (drive stays disabled) the whole time.
 *
 * Build + run: see igh/README.md  (needs the IgH master installed + running).
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include <sched.h>

#include "ecrt.h"

#define VENDOR      0x00400000u   /* ANCTL           (ec_inspect: man)  */
#define PRODUCT     0x00000715u   /* AS715N          (ec_inspect: id)   */
#define CYCLE_NS    2000000L      /* 2 ms SYNC0                          */
#define SYNC0_SHIFT 0             /* tune if needed (e.g. CYCLE_NS/2)    */
#define ASSIGN_ACT  0x0300        /* DC AssignActivate (SYNC0); from ESI */
#define RUN_SECONDS 12

#define NSEC_PER_SEC 1000000000L
#define TS2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

static volatile sig_atomic_t running = 1;
static void on_sig(int s) { (void)s; running = 0; }

static ec_master_t *master;
static ec_domain_t *domain;
static uint8_t *pd;
static ec_slave_config_t *sc;

/* RxPDO 0x1701 (PC->drive) and TxPDO 0x1B01 (drive->PC) — from ec_inspect.py. */
static ec_pdo_entry_info_t rx_entries[] = {
    {0x6040, 0, 16}, {0x607A, 0, 32}, {0x60B8, 0, 16}, {0x60FE, 1, 32},
};
static ec_pdo_entry_info_t tx_entries[] = {
    {0x603F, 0, 16}, {0x6041, 0, 16}, {0x6064, 0, 32}, {0x6077, 0, 16},
    {0x60F4, 0, 32}, {0x60B9, 0, 16}, {0x60BA, 0, 32}, {0x60BC, 0, 32},
    {0x60FD, 0, 32},
};
static ec_pdo_info_t rx_pdos[] = {{0x1701, 4, rx_entries}};
static ec_pdo_info_t tx_pdos[] = {{0x1B01, 9, tx_entries}};
static ec_sync_info_t syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, rx_pdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, tx_pdos, EC_WD_DISABLE},
    {0xff}
};

/* byte offsets of the entries we read/write in the process image */
static unsigned int o_ctrl, o_target;               /* out */
static unsigned int o_err, o_status, o_pos;         /* in  */

static ec_pdo_entry_reg_t regs[] = {
    {0, 0, VENDOR, PRODUCT, 0x6040, 0, &o_ctrl},
    {0, 0, VENDOR, PRODUCT, 0x607A, 0, &o_target},
    {0, 0, VENDOR, PRODUCT, 0x603F, 0, &o_err},
    {0, 0, VENDOR, PRODUCT, 0x6041, 0, &o_status},
    {0, 0, VENDOR, PRODUCT, 0x6064, 0, &o_pos},
    {}
};

int main(void)
{
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    master = ecrt_request_master(0);
    if (!master) { fprintf(stderr, "ecrt_request_master failed (is the IgH master running?)\n"); return 1; }

    domain = ecrt_master_create_domain(master);
    if (!domain) { fprintf(stderr, "create_domain failed\n"); return 1; }

    sc = ecrt_master_slave_config(master, 0, 0, VENDOR, PRODUCT);
    if (!sc) { fprintf(stderr, "slave_config failed (vendor/product mismatch?)\n"); return 1; }

    if (ecrt_slave_config_pdos(sc, EC_END, syncs)) {
        fprintf(stderr, "config_pdos failed\n"); return 1;
    }
    /* Mode of operation -> CSP (8), set once over SDO (not in this PDO map). */
    ecrt_slave_config_sdo8(sc, 0x6060, 0, 8);

    /* THE point of the test: DC/SYNC0 configured by IgH. */
    ecrt_slave_config_dc(sc, ASSIGN_ACT, CYCLE_NS, SYNC0_SHIFT, 0, 0);

    if (ecrt_domain_reg_pdo_entry_list(domain, regs)) {
        fprintf(stderr, "reg_pdo_entry_list failed\n"); return 1;
    }
    if (ecrt_master_activate(master)) { fprintf(stderr, "activate failed\n"); return 1; }
    if (!(pd = ecrt_domain_data(domain))) { fprintf(stderr, "domain_data failed\n"); return 1; }

    mlockall(MCL_CURRENT | MCL_FUTURE);
    struct sched_param sp = { .sched_priority = 80 };
    sched_setscheduler(0, SCHED_FIFO, &sp);

    struct timespec wakeup;
    clock_gettime(CLOCK_MONOTONIC, &wakeup);

    long cycles = 0, last_print = -1;
    time_t t0 = time(NULL);

    while (running) {
        wakeup.tv_nsec += CYCLE_NS;
        while (wakeup.tv_nsec >= NSEC_PER_SEC) { wakeup.tv_nsec -= NSEC_PER_SEC; wakeup.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);

        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        uint16_t status = EC_READ_U16(pd + o_status);
        uint16_t err    = EC_READ_U16(pd + o_err);
        int32_t  pos    = EC_READ_S32(pd + o_pos);

        EC_WRITE_U16(pd + o_ctrl, 0x0000);           /* stay disabled */
        EC_WRITE_S32(pd + o_target, pos);            /* hold, no jump */

        /* Distributed clocks: give IgH the app time and sync the clocks. */
        ecrt_master_application_time(master, TS2NS(wakeup));
        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        long sec = time(NULL) - t0;
        if (sec != last_print) {
            last_print = sec;
            ec_master_state_t ms;
            ecrt_master_state(master, &ms);
            printf("t=%2lds  status=0x%04X  err=0x%04X  pos=%d  al_states=0x%02X  cycles=%ld\n",
                   sec, status, err, pos, ms.al_states, cycles);
            if (sec >= RUN_SECONDS) break;
        }
        cycles++;
    }

    printf("\n%s\n", "done — err=0x0000 throughout means IgH DC works (path proven); "
                     "err=0x8700 means the sync fault persists even on IgH.");
    ecrt_master_deactivate(master);
    ecrt_release_master(master);
    return 0;
}
