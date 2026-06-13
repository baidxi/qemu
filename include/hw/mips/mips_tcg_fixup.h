/*
 * MIPS TCG instruction fixup API.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MIPS_TCG_FIXUP_H
#define MIPS_TCG_FIXUP_H

#include "qemu/osdep.h"
#include "target/mips/cpu.h"

/**
 * mips_tcg_fixup: Register a single instruction fixup.
 *
 * When the TCG translator encounters the instruction at @vaddr whose raw
 * opcode matches @orig, it transparently replaces it with @new before
 * decoding.  This is intended for machine-specific firmware workarounds
 * where the problematic instruction is generated at runtime (e.g. by a
 * decompressing bootloader) and therefore cannot be patched in the
 * firmware image.
 *
 * Only one fixup can be active at a time.  Calling this function a second
 * time replaces the previous fixup.  Pass @vaddr == 0 to disable.
 */
void mips_tcg_fixup(target_ulong vaddr, uint32_t orig, uint32_t neu);

/**
 * mips_tcg_fixup_pattern: Register a version-independent pattern-based fixup.
 *
 * When the TCG translator encounters an instruction whose raw opcode matches
 * @target_opcode at ANY address, it reads the instruction at @ctx_offset
 * bytes relative to the current PC.  If that instruction matches
 * @ctx_opcode, the target instruction is replaced with @new.
 *
 * This is version-independent: the fix is applied based on the surrounding
 * instruction sequence, not a hardcoded address.  Used for firmware bugs
 * where the buggy instruction appears at different addresses across versions
 * but always in the same instruction context.
 *
 * Example (Breed dlmalloc non-contiguous top-chunk corruption):
 *   The buggy "sw $v1, 4($v0)" (0xAC430004) is preceded two instructions
 *   earlier by "li $v1, 1" (0x24030001).  This pattern is unique across
 *   the entire firmware, making the fix portable across Breed versions.
 */
void mips_tcg_fixup_pattern(uint32_t target_opcode, uint32_t neu,
                             uint32_t ctx_opcode, int ctx_offset);

#endif /* MIPS_TCG_FIXUP_H */
