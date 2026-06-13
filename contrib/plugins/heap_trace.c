/*
 * QEMU TCG Plugin: Heap corruption tracer for Breed dlmalloc
 *
 * Uses memory access callbacks (no register API needed) to trace
 * the critical store instructions in malloc's non-contiguous MORECORE path.
 *
 * Watches:
 *   0x8ffc22d0: sw $18, 8($16)   → stores NEW top ptr to mstate
 *   0x8ffc22e4: sw $2, 4($18)    → stores NEW top size (correct)
 *   0x8ffc2300: sw $3, 4($2)     → BUG: corrupts NEW top size to 1
 *   brk read/write in MORECORE and malloc to detect desync
 *
 * Build: see contrib/plugins/meson.build
 */
#include <glib.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

/* Key addresses in Breed (kseg0) */
#define SBRK_ENTRY       0x8ffc18f0
#define SBRK_RETURN      0x8ffc1988
#define MALLOC_NONCONTIG 0x8ffc2254  /* Non-contiguous MORECORE path entry */
#define MALLOC_SET_TOP   0x8ffc22d0  /* sw $18, 8($16) - set new top ptr */
#define MALLOC_SET_SIZE  0x8ffc22e4  /* sw $2, 4($18) - set new top size (correct) */
#define MALLOC_BUG_STORE 0x8ffc2300  /* sw $3, 4($2) - BUG: corrupt new top */
#define MALLOC_DUMMY_CHK 0x8ffc22e0  /* beq $19, $16 - dummy check */
#define MALLOC_SIZE_CHK  0x8ffc22e8  /* sltiu $2, $20, 0x10 - size < 16 check */

/* brk variable tracking */
#define MORECORE_BRK_READ  0x8ffc191c /* lw $17, 0($2) - MORECORE reads its brk */
#define MORECORE_BRK_WRITE 0x8ffc1970 /* sw $16, 0($18) - MORECORE writes its brk */
#define MALLOC_BRK_READ    0x8ffc2230 /* lw $2, 0x4564($22) - malloc reads brk */
#define MALLOC_BRK_WRITE_C 0x8ffc223c /* sw $2, 0x4564($22) - contig brk update */
#define MALLOC_BRK_WRITE_N 0x8ffc22cc /* sw $3, 0x4564($22) - noncontig brk update */

#define ERROR_LOOP       0x8ffbeb18

static int sbrk_count;
static int noncontig_count;
static int bug_trigger_count;
static int error_loop_count;

/* Memory callback for memory access instructions - captures addr and value */
static void mem_cb(unsigned int cpu_index, qemu_plugin_meminfo_t info,
                   uint64_t vaddr, void *udata)
{
    uint64_t pc = (uint64_t)(uintptr_t)udata;
    int is_store = qemu_plugin_mem_is_store(info);
    qemu_plugin_mem_value val = qemu_plugin_mem_get_value(info);
    uint64_t mem_val = 0;

    switch (val.type) {
    case QEMU_PLUGIN_MEM_VALUE_U8:  mem_val = val.data.u8;  break;
    case QEMU_PLUGIN_MEM_VALUE_U16: mem_val = val.data.u16; break;
    case QEMU_PLUGIN_MEM_VALUE_U32: mem_val = val.data.u32; break;
    case QEMU_PLUGIN_MEM_VALUE_U64: mem_val = val.data.u64; break;
    default: break;
    }

    if (pc == MALLOC_SET_TOP) {
        fprintf(stderr, "  [SET_TOP] *(0x%08llx) = 0x%08llx  (new top ptr)\n",
                (unsigned long long)vaddr, (unsigned long long)mem_val);
    } else if (pc == MALLOC_SET_SIZE) {
        fprintf(stderr, "  [SET_SIZE] *(0x%08llx) = 0x%08llx  (new top size, CORRECT)\n",
                (unsigned long long)vaddr, (unsigned long long)mem_val);
    } else if (pc == MALLOC_BUG_STORE) {
        bug_trigger_count++;
        fprintf(stderr, "  [BUG!!] *(0x%08llx) = 0x%08llx  - CORRUPTS NEW TOP SIZE!\n",
                (unsigned long long)vaddr, (unsigned long long)mem_val);
    } else if (pc == MORECORE_BRK_READ && !is_store) {
        fprintf(stderr, "    MCORE brk READ: *(0x%08llx) = 0x%08llx\n",
                (unsigned long long)vaddr, (unsigned long long)mem_val);
    } else if (pc == MORECORE_BRK_WRITE && is_store) {
        fprintf(stderr, "    MCORE brk WRITE: *(0x%08llx) = 0x%08llx\n",
                (unsigned long long)vaddr, (unsigned long long)mem_val);
    } else if (pc == MALLOC_BRK_READ && !is_store) {
        fprintf(stderr, "    MALLOC brk READ: *(0x%08llx) = 0x%08llx\n",
                (unsigned long long)vaddr, (unsigned long long)mem_val);
    } else if ((pc == MALLOC_BRK_WRITE_C || pc == MALLOC_BRK_WRITE_N) && is_store) {
        const char *path = (pc == MALLOC_BRK_WRITE_C) ? "CONTIG" : "NONCONTIG";
        fprintf(stderr, "    MALLOC brk WRITE(%s): *(0x%08llx) = 0x%08llx\n",
                path, (unsigned long long)vaddr, (unsigned long long)mem_val);
    }
}

/* Exec callback for non-contiguous path entry */
static void noncontig_cb(unsigned int cpu_index, void *udata)
{
    noncontig_count++;
    fprintf(stderr, "[%d] >>> NON-CONTIGUOUS MORECORE RETURN\n", noncontig_count);
}

static void dummy_chk_cb(unsigned int cpu_index, void *udata)
{
    fprintf(stderr, "  [DUMMY_CHK] old_top vs mstate\n");
}

static void size_chk_cb(unsigned int cpu_index, void *udata)
{
    fprintf(stderr, "  [SIZE_CHK] old_top_size < 16\n");
}

static void sbrk_entry_cb(unsigned int cpu_index, void *udata)
{
    fprintf(stderr, "[%d] SBRK call\n", sbrk_count);
    sbrk_count++;
}

static void sbrk_return_cb(unsigned int cpu_index, void *udata)
{
    fprintf(stderr, "  SBRK return\n");
}

static void error_loop_cb(unsigned int cpu_index, void *udata)
{
    if (error_loop_count < 3) {
        fprintf(stderr, "!!! ERROR LOOP !!!\n");
    }
    error_loop_count++;
}

static void vcpu_tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    size_t n_insns = qemu_plugin_tb_n_insns(tb);

    for (size_t i = 0; i < n_insns; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t vaddr = qemu_plugin_insn_vaddr(insn);

        if (vaddr == SBRK_ENTRY) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, sbrk_entry_cb, QEMU_PLUGIN_CB_NO_REGS, NULL);
        } else if (vaddr == SBRK_RETURN) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, sbrk_return_cb, QEMU_PLUGIN_CB_NO_REGS, NULL);
        } else if (vaddr == MALLOC_NONCONTIG) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, noncontig_cb, QEMU_PLUGIN_CB_NO_REGS, NULL);
        } else if (vaddr == MALLOC_DUMMY_CHK) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, dummy_chk_cb, QEMU_PLUGIN_CB_NO_REGS, NULL);
        } else if (vaddr == MALLOC_SIZE_CHK) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, size_chk_cb, QEMU_PLUGIN_CB_NO_REGS, NULL);
        } else if (vaddr == ERROR_LOOP) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, error_loop_cb, QEMU_PLUGIN_CB_NO_REGS, NULL);
        }
        /* Memory callbacks for store/load instructions */
        else if (vaddr == MALLOC_SET_TOP) {
            qemu_plugin_register_vcpu_mem_cb(
                insn, mem_cb, QEMU_PLUGIN_CB_NO_REGS,
                QEMU_PLUGIN_MEM_W, (void *)(uintptr_t)MALLOC_SET_TOP);
        } else if (vaddr == MALLOC_SET_SIZE) {
            qemu_plugin_register_vcpu_mem_cb(
                insn, mem_cb, QEMU_PLUGIN_CB_NO_REGS,
                QEMU_PLUGIN_MEM_W, (void *)(uintptr_t)MALLOC_SET_SIZE);
        } else if (vaddr == MALLOC_BUG_STORE) {
            qemu_plugin_register_vcpu_mem_cb(
                insn, mem_cb, QEMU_PLUGIN_CB_NO_REGS,
                QEMU_PLUGIN_MEM_W, (void *)(uintptr_t)MALLOC_BUG_STORE);
        } else if (vaddr == MORECORE_BRK_READ) {
            qemu_plugin_register_vcpu_mem_cb(
                insn, mem_cb, QEMU_PLUGIN_CB_NO_REGS,
                QEMU_PLUGIN_MEM_R, (void *)(uintptr_t)MORECORE_BRK_READ);
        } else if (vaddr == MORECORE_BRK_WRITE) {
            qemu_plugin_register_vcpu_mem_cb(
                insn, mem_cb, QEMU_PLUGIN_CB_NO_REGS,
                QEMU_PLUGIN_MEM_W, (void *)(uintptr_t)MORECORE_BRK_WRITE);
        } else if (vaddr == MALLOC_BRK_READ) {
            qemu_plugin_register_vcpu_mem_cb(
                insn, mem_cb, QEMU_PLUGIN_CB_NO_REGS,
                QEMU_PLUGIN_MEM_R, (void *)(uintptr_t)MALLOC_BRK_READ);
        } else if (vaddr == MALLOC_BRK_WRITE_C) {
            qemu_plugin_register_vcpu_mem_cb(
                insn, mem_cb, QEMU_PLUGIN_CB_NO_REGS,
                QEMU_PLUGIN_MEM_W, (void *)(uintptr_t)MALLOC_BRK_WRITE_C);
        } else if (vaddr == MALLOC_BRK_WRITE_N) {
            qemu_plugin_register_vcpu_mem_cb(
                insn, mem_cb, QEMU_PLUGIN_CB_NO_REGS,
                QEMU_PLUGIN_MEM_W, (void *)(uintptr_t)MALLOC_BRK_WRITE_N);
        }
    }
}

static void plugin_exit(qemu_plugin_id_t id, void *p)
{
    fprintf(stderr, "\n=== Heap Trace Summary ===\n");
    fprintf(stderr, "sbrk calls:          %d\n", sbrk_count);
    fprintf(stderr, "non-contig events:   %d\n", noncontig_count);
    fprintf(stderr, "BUG trigger count:   %d\n", bug_trigger_count);
    fprintf(stderr, "error loop count:    %d\n", error_loop_count);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                             const qemu_info_t *info,
                                             int argc, char **argv)
{
    fprintf(stderr, "Heap corruption tracer loaded\n");
    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
