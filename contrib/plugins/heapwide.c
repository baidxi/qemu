/*
 * QEMU TCG Plugin: Wide heap store tracer for Breed dlmalloc corruption
 *
 * Watches ALL stores to the heap area 0x8fdbc000-0x8fdd8000 to find
 * what writes 0x00000001 at every page boundary.
 *
 * Build:
 *   gcc -shared -fPIC -I include -o build/heapwide.so contrib/plugins/heapwide.c
 *
 * Usage:
 *   -plugin ./build/heapwide.so
 */
#include <glib.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

/* Watch the entire heap area where corruption happens */
#define WATCH_START  0x8fdbc000ULL
#define WATCH_END    0x8fdd8000ULL

static FILE *logfile;
static GMutex lock;
static long store_count;
static int page4_stores_only = 1; /* Only log stores to offset 0x4 of each page */

static inline int in_range(uint64_t vaddr)
{
    return vaddr >= WATCH_START && vaddr < WATCH_END;
}

static void mem_cb(unsigned int cpu_index, qemu_plugin_meminfo_t info,
                   uint64_t vaddr, void *udata)
{
    if (!qemu_plugin_mem_is_store(info)) return;
    if (!in_range(vaddr)) return;

    /* Filter: only show stores to page offset 0x4 (where 0x1 appears) */
    if (page4_stores_only && (vaddr & 0xfff) != 0x004) return;

    uint64_t pc = (uint64_t)(uintptr_t)udata;

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

    g_mutex_lock(&lock);
    store_count++;
    fprintf(logfile,
            "STORE[%ld] cpu=%u pc=0x%08llx va=0x%08llx %s=0x%llx\n",
            store_count, cpu_index,
            (unsigned long long)pc, (unsigned long long)vaddr,
            sz, (unsigned long long)store_val);
    fflush(logfile);
    g_mutex_unlock(&lock);
}

static void vcpu_tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    size_t n = qemu_plugin_tb_n_insns(tb);
    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t pc = qemu_plugin_insn_vaddr(insn);

        qemu_plugin_register_vcpu_mem_cb(insn, mem_cb,
                                         QEMU_PLUGIN_CB_NO_REGS,
                                         QEMU_PLUGIN_MEM_W,
                                         (void *)(uintptr_t)pc);
    }
}

static void plugin_exit(qemu_plugin_id_t id, void *p)
{
    g_mutex_lock(&lock);
    fprintf(logfile, "\n=== Total stores logged: %ld ===\n", store_count);
    fflush(logfile);
    fclose(logfile);
    g_mutex_unlock(&lock);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                            const qemu_info_t *info,
                                            int argc, char **argv)
{
    logfile = fopen("/tmp/heapwide.log", "w");
    if (!logfile) {
        fprintf(stderr, "Cannot open /tmp/heapwide.log\n");
        return -1;
    }
    fprintf(logfile, "# Heap wide store tracer: 0x%08llx - 0x%08llx\n",
            (unsigned long long)WATCH_START, (unsigned long long)WATCH_END);
    fprintf(logfile, "# Filtering: page offset 0x4 stores only\n\n");
    fflush(logfile);

    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
