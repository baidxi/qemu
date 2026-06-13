/*
 * QEMU TCG Plugin: MT7621 Breed malloc/fifo tracer
 *
 * Traces the malloc/sbrk/fifo allocation chain to understand
 * why "unable to allocate fifos" occurs.
 *
 * Build: see contrib/plugins/meson.build
 */

#include <glib.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

/* Key addresses in Breed (kseg0) */
#define DLMALLOC_ENTRY  0x8ffc1d3c
#define SBRK_ENTRY      0x8ffc18f0
#define SBRK_RETURN     0x8ffc1988
#define CALLOC_ENTRY    0x8ffc2420
#define FIFO_ALLOC_ENT  0x8ffc549c
#define LI_A0_240       0x8ffc54dc
#define JALR_GOT44      0x8ffc54e4
#define AFTER_GOT44     0x8ffc54ec
#define ERROR_LOOP      0x8ffbeb18

static int dlmalloc_count;
static int sbrk_count;
static int calloc_count;
static int fifo_alloc_count;
static int li_a0_240_count;

/* Register handles - cached per vCPU */
typedef struct {
    struct qemu_plugin_register *a0_handle;
    struct qemu_plugin_register *a1_handle;
    struct qemu_plugin_register *v0_handle;
    struct qemu_plugin_register *t9_handle;
    struct qemu_plugin_register *gp_handle;
    struct qemu_plugin_register *sp_handle;
    struct qemu_plugin_register *ra_handle;
    bool initialized;
} VCPURegs;

static VCPURegs vcpu_regs[1]; /* single CPU */

static void init_regs(unsigned int cpu_index)
{
    VCPURegs *r = &vcpu_regs[cpu_index];
    if (r->initialized) return;

    GArray *regs = qemu_plugin_get_registers();
    if (!regs) {
        fprintf(stderr, "PLUGIN: qemu_plugin_get_registers() returned NULL!\n");
        return;
    }

    fprintf(stderr, "PLUGIN: Found %d register classes\n", regs->len);
    for (guint i = 0; i < regs->len && i < 40; i++) {
        qemu_plugin_reg_descriptor *desc =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);
        fprintf(stderr, "  reg[%d]: name='%s' feature='%s'\n", i, desc->name, desc->feature);
        if (strcmp(desc->name, "a0") == 0) r->a0_handle = desc->handle;
        else if (strcmp(desc->name, "a1") == 0) r->a1_handle = desc->handle;
        else if (strcmp(desc->name, "v0") == 0) r->v0_handle = desc->handle;
        else if (strcmp(desc->name, "t9") == 0) r->t9_handle = desc->handle;
        else if (strcmp(desc->name, "gp") == 0) r->gp_handle = desc->handle;
        else if (strcmp(desc->name, "sp") == 0) r->sp_handle = desc->handle;
        else if (strcmp(desc->name, "ra") == 0) r->ra_handle = desc->handle;
        /* Also try alternate names */
        else if (strcmp(desc->name, "r4") == 0) r->a0_handle = desc->handle;
        else if (strcmp(desc->name, "r5") == 0) r->a1_handle = desc->handle;
        else if (strcmp(desc->name, "r2") == 0) r->v0_handle = desc->handle;
        else if (strcmp(desc->name, "r25") == 0) r->t9_handle = desc->handle;
        else if (strcmp(desc->name, "r28") == 0) r->gp_handle = desc->handle;
        else if (strcmp(desc->name, "r29") == 0) r->sp_handle = desc->handle;
        else if (strcmp(desc->name, "r31") == 0) r->ra_handle = desc->handle;
    }
    r->initialized = true;
    g_array_free(regs, TRUE);
    fprintf(stderr, "PLUGIN: a0=%p a1=%p v0=%p t9=%p gp=%p sp=%p ra=%p\n",
            (void*)r->a0_handle, (void*)r->a1_handle, (void*)r->v0_handle,
            (void*)r->t9_handle, (void*)r->gp_handle, (void*)r->sp_handle,
            (void*)r->ra_handle);
}

static uint32_t read_reg32(unsigned int cpu_index,
                           struct qemu_plugin_register *handle)
{
    if (!handle) return 0;
    GByteArray *buf = g_byte_array_new();
    bool ok = qemu_plugin_read_register(handle, buf);
    uint32_t val = 0;
    if (ok && buf->len >= 4) {
        memcpy(&val, buf->data, 4);
    }
    g_byte_array_free(buf, TRUE);
    return val;
}

/* dlmalloc internal entry: log a0 (requested size), gp, GOT+36 */
static void dlmalloc_entry_cb(unsigned int cpu_index, void *udata)
{
    init_regs(cpu_index);
    uint32_t a0 = read_reg32(cpu_index, vcpu_regs[cpu_index].a0_handle);
    uint32_t gp = read_reg32(cpu_index, vcpu_regs[cpu_index].gp_handle);
    uint32_t ra = read_reg32(cpu_index, vcpu_regs[cpu_index].ra_handle);
    fprintf(stderr, "[%d] DLMALLOC(0x8FFC1D3C): size=0x%x (%d) gp=0x%x ra=0x%x\n",
            dlmalloc_count, a0, (int32_t)a0, gp, ra);
    /* Log key GOT entries */
    fprintf(stderr, "  GOT+36(mstate)=*(0x%x) GOT+624(brk_pp)=*(0x%x) GOT+628(stk)=*(0x%x)\n",
            gp + 36, gp + 624, gp + 628);
    dlmalloc_count++;
}

/* sbrk entry: log a0 (increment) */
static void sbrk_entry_cb(unsigned int cpu_index, void *udata)
{
    init_regs(cpu_index);
    uint32_t a0 = read_reg32(cpu_index, vcpu_regs[cpu_index].a0_handle);
    fprintf(stderr, "[%d] SBRK(0x8FFC18F0): increment=0x%x (%d)\n",
            sbrk_count, a0, (int32_t)a0);
    sbrk_count++;
}

/* sbrk return (jr ra): log v0 (return value) */
static void sbrk_return_cb(unsigned int cpu_index, void *udata)
{
    init_regs(cpu_index);
    uint32_t v0 = read_reg32(cpu_index, vcpu_regs[cpu_index].v0_handle);
    fprintf(stderr, "    SBRK RETURN: v0=0x%x (%s)\n",
            v0, v0 == 0xFFFFFFFF ? "FAIL(-1)" : "old_break");
}

/* calloc entry: log a0 (count) and a1 (size) */
static void calloc_entry_cb(unsigned int cpu_index, void *udata)
{
    init_regs(cpu_index);
    uint32_t a0 = read_reg32(cpu_index, vcpu_regs[cpu_index].a0_handle);
    uint32_t a1 = read_reg32(cpu_index, vcpu_regs[cpu_index].a1_handle);
    fprintf(stderr, "[%d] CALLOC(0x8FFC2420): count=0x%x size=0x%x total=0x%x\n",
            calloc_count, a0, a1, a0 * a1);
    calloc_count++;
}

/* fifo allocator entry */
static void fifo_alloc_entry_cb(unsigned int cpu_index, void *udata)
{
    fprintf(stderr, "[%d] FIFO_ALLOC(0x8FFC549C) entry\n", fifo_alloc_count);
    fifo_alloc_count++;
}

/* li a0,240 instruction reached */
static void li_a0_240_cb(unsigned int cpu_index, void *udata)
{
    fprintf(stderr, "[%d] LI_A0_240(0x8FFC54DC): about to call GOT[44] with a0=240\n",
            li_a0_240_count);
    li_a0_240_count++;
}

/* jalr t9 at 0x8FFC54E4: log t9 (target function address) */
static void jalr_got44_cb(unsigned int cpu_index, void *udata)
{
    init_regs(cpu_index);
    uint32_t t9 = read_reg32(cpu_index, vcpu_regs[cpu_index].t9_handle);
    uint32_t a0 = read_reg32(cpu_index, vcpu_regs[cpu_index].a0_handle);
    fprintf(stderr, "  JALR GOT[44]: t9=0x%x a0=0x%x (%d)\n",
            t9, a0, (int32_t)a0);
}

/* After GOT[44] call returns: log v0 */
static void after_got44_cb(unsigned int cpu_index, void *udata)
{
    init_regs(cpu_index);
    uint32_t v0 = read_reg32(cpu_index, vcpu_regs[cpu_index].v0_handle);
    fprintf(stderr, "  GOT[44] returned: v0=0x%x (%s)\n",
            v0, v0 ? "SUCCESS" : "NULL/FAIL");
}

/* Error loop reached */
static void error_loop_cb(unsigned int cpu_index, void *udata)
{
    fprintf(stderr, "!!! ERROR LOOP REACHED (0x8FFBEB18) !!!\n");
}

static void vcpu_tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    size_t n_insns = qemu_plugin_tb_n_insns(tb);

    for (size_t i = 0; i < n_insns; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t vaddr = qemu_plugin_insn_vaddr(insn);

        if (vaddr == DLMALLOC_ENTRY) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, dlmalloc_entry_cb,
                QEMU_PLUGIN_CB_R_REGS, NULL);
        } else if (vaddr == SBRK_ENTRY) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, sbrk_entry_cb,
                QEMU_PLUGIN_CB_R_REGS, NULL);
        } else if (vaddr == SBRK_RETURN) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, sbrk_return_cb,
                QEMU_PLUGIN_CB_R_REGS, NULL);
        } else if (vaddr == CALLOC_ENTRY) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, calloc_entry_cb,
                QEMU_PLUGIN_CB_R_REGS, NULL);
        } else if (vaddr == FIFO_ALLOC_ENT) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, fifo_alloc_entry_cb,
                QEMU_PLUGIN_CB_NO_REGS, NULL);
        } else if (vaddr == LI_A0_240) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, li_a0_240_cb,
                QEMU_PLUGIN_CB_NO_REGS, NULL);
        } else if (vaddr == JALR_GOT44) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, jalr_got44_cb,
                QEMU_PLUGIN_CB_R_REGS, NULL);
        } else if (vaddr == AFTER_GOT44) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, after_got44_cb,
                QEMU_PLUGIN_CB_R_REGS, NULL);
        } else if (vaddr == ERROR_LOOP) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, error_loop_cb,
                QEMU_PLUGIN_CB_NO_REGS, NULL);
        }
    }
}

static void plugin_exit(qemu_plugin_id_t id, void *p)
{
    fprintf(stderr, "\n=== MT7621 Debug Summary ===\n");
    fprintf(stderr, "dlmalloc calls:  %d\n", dlmalloc_count);
    fprintf(stderr, "sbrk calls:      %d\n", sbrk_count);
    fprintf(stderr, "calloc calls:    %d\n", calloc_count);
    fprintf(stderr, "fifo_alloc calls:%d\n", fifo_alloc_count);
    fprintf(stderr, "li_a0_240 calls: %d\n", li_a0_240_count);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                            const qemu_info_t *info,
                                            int argc, char **argv)
{
    fprintf(stderr, "MT7621 Breed malloc/fifo tracer loaded\n");

    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);

    return 0;
}
