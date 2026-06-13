/*
 * QEMU TCG Plugin: Heap write tracer for MT7621 Breed dlmalloc
 *
 * Traces all store operations to the mstate structure and heap metadata
 * area to find what corrupts the top chunk.
 *
 * Build:
 *   gcc -shared -fPIC -I include -o build/heaptrace.so contrib/plugins/heaptrace.c
 *
 * Usage:
 *   -plugin ./build/heaptrace.so
 */

#include <glib.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

/*
 * Ranges to watch (KSEG0 virtual addresses).
 * mstate:    0x8FFF0120 .. 0x8FFF0160  (top ptr, smallmap, dv, etc.)
 * gap area:  0x8FDBB500 .. 0x8FDBB5B8  (where chunk walk stops)
 * top chunk: 0x8FDD6FF0 .. 0x8FDD7010  (top chunk header area)
 */
#define MSTATE_START   0x8FFF0120ULL
#define MSTATE_END     0x8FFF0180ULL
#define GAP_START      0x8FDBB500ULL
#define GAP_END        0x8FDBB5C0ULL
#define TOP_START      0x8FDD6FF0ULL
#define TOP_END        0x8FDD7010ULL
/* Also watch the global struct heap fields */
#define GSTRUCT_START  0x8FFFD560ULL
#define GSTRUCT_END    0x8FFFD5B0ULL

static FILE *logfile;
static GMutex lock;
static long store_count;

static inline int in_range(uint64_t vaddr)
{
    return (vaddr >= MSTATE_START && vaddr < MSTATE_END) ||
           (vaddr >= GAP_START && vaddr < GAP_END) ||
           (vaddr >= TOP_START && vaddr < TOP_END) ||
           (vaddr >= GSTRUCT_START && vaddr < GSTRUCT_END);
}

static const char *range_name(uint64_t vaddr)
{
    if (vaddr >= MSTATE_START && vaddr < MSTATE_END) return "MSTATE";
    if (vaddr >= GAP_START && vaddr < GAP_END) return "GAP";
    if (vaddr >= TOP_START && vaddr < TOP_END) return "TOP";
    if (vaddr >= GSTRUCT_START && vaddr < GSTRUCT_END) return "GSTRUCT";
    return "?";
}

static void mem_cb(unsigned int cpu_index, qemu_plugin_meminfo_t info,
                   uint64_t vaddr, void *udata)
{
    if (!qemu_plugin_mem_is_store(info)) return;
    if (!in_range(vaddr)) return;

    uint64_t pc = (uint64_t)(uintptr_t)udata;

    /* Get the value being stored */
    qemu_plugin_mem_value val = qemu_plugin_mem_get_value(info);
    uint64_t store_val = 0;
    const char *sz = "?";
    switch (val.type) {
    case QEMU_PLUGIN_MEM_VALUE_U8:  store_val = val.data.u8;  sz = "u8";  break;
    case QEMU_PLUGIN_MEM_VALUE_U16: store_val = val.data.u16; sz = "u16"; break;
    case QEMU_PLUGIN_MEM_VALUE_U32: store_val = val.data.u32; sz = "u32"; break;
    case QEMU_PLUGIN_MEM_VALUE_U64: store_val = val.data.u64; sz = "u64"; break;
    default: break;
    }

    /* Get physical address */
    struct qemu_plugin_hwaddr *hwaddr = qemu_plugin_get_hwaddr(info, vaddr);
    uint64_t phys = hwaddr ? qemu_plugin_hwaddr_phys_addr(hwaddr) : 0;

    g_mutex_lock(&lock);
    store_count++;
    fprintf(logfile,
            "STORE[%ld] %-7s cpu=%u pc=0x%08llx va=0x%08llx pa=0x%08llx %s=0x%llx\n",
            store_count, range_name(vaddr), cpu_index,
            (unsigned long long)pc, (unsigned long long)vaddr,
            (unsigned long long)phys, sz, (unsigned long long)store_val);
    fflush(logfile);
    g_mutex_unlock(&lock);
}

static void vcpu_tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    size_t n = qemu_plugin_tb_n_insns(tb);
    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t pc = qemu_plugin_insn_vaddr(insn);
        /* Only register for write accesses to reduce overhead */
        qemu_plugin_register_vcpu_mem_cb(insn, mem_cb,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         QEMU_PLUGIN_MEM_W,
                                         (void *)(uintptr_t)pc);
    }
}

static void plugin_exit(qemu_plugin_id_t id, void *p)
{
    if (logfile) {
        fprintf(logfile, "\n=== Total stores logged: %ld ===\n", store_count);
        fclose(logfile);
    }
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    logfile = fopen("/tmp/heap_stores.log", "w");
    if (!logfile) {
        fprintf(stderr, "heaptrace: cannot open /tmp/heap_stores.log\n");
        return -1;
    }
    fprintf(logfile, "# Heap write trace - mstate/gap/top/gstruct ranges\n");
    fflush(logfile);

    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
