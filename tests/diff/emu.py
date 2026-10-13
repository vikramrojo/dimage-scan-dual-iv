#!/usr/bin/env python3
"""
Generic PowerPC/CFM emulation harness: loads one of the original Konica
Minolta PEF libraries (e.g. MFSLib2891) into the Unicorn emulator and calls
arbitrary internal functions with prepared memory, so a native port can be
compared against the original output by output.

    from emu import Emu
    m = Emu('/path/to/MFSLib2891')
    buf = m.alloc(1024)
    r3, f1 = m.call(0x10036090, [in_ptr, out_ptr])          # Ghidra address
    r3, f1 = m.call('GetStdGain', [...], fargs=[1.0])        # traceback name

Addresses may be given in Ghidra's layout (code section at 0x10000000, as in
the decompiled .c files) or as traceback names.

Calling convention (CFM / PowerOpen): integer args in r3..r10, doubles in
f1..f13.  Each double also consumes two argument words, so mixed signatures
are best passed with call_words(), which takes the raw parameter-area words
(first 8 in r3..r10, the rest on the stack at SP+56) plus the FPR list.
"""
import math
import os
import struct
import sys

from unicorn import Uc, UC_ARCH_PPC, UC_MODE_PPC32, UC_MODE_BIG_ENDIAN, UC_HOOK_CODE, UcError
from unicorn.ppc_const import *

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'docs', 're-tools'))
from pef import PEF  # noqa: E402

TRAP_BASE = 0x0F000000
TRAP_CODE = TRAP_BASE + 0x10000
RET_ADDR = TRAP_BASE + 0x1F000
STACK_TOP = 0x7F0F0000
HEAP_BASE = 0x60000000
HEAP_SIZE = 0x10000000
GHIDRA_CODE = 0x10000000


class Emu:
    def __init__(self, lib, extra_imports=None):
        self.p = PEF(lib)
        self.p.find_functions()
        self.uc = uc = Uc(UC_ARCH_PPC, UC_MODE_PPC32 | UC_MODE_BIG_ENDIAN)
        try:
            uc.ctl_set_cpu_model(UC_CPU_PPC32_7400_V2_9)
        except Exception:
            pass
        for s in self.p.secs:
            if s['kind'] in (0, 1, 2, 3, 6):
                base = s['base'] & ~0xFFF
                size = ((s['base'] + max(s['total'], 1) - base) + 0xFFF) & ~0xFFF
                uc.mem_map(base, size)
                uc.mem_write(s['base'], bytes(s['body']))
        self.code_base = self.p.secs[self.p.code_idx[0]]['base']
        uc.mem_map(TRAP_BASE, 0x20000)
        uc.mem_write(TRAP_CODE, struct.pack('>I', 0x4E800020) * (0x10000 // 4))
        uc.mem_map(STACK_TOP - 0x200000, 0x210000)
        uc.mem_map(HEAP_BASE, HEAP_SIZE)
        self.heap = HEAP_BASE + 0x1000
        tocs = {}
        for a, (k, v) in self.p.annot.items():
            if k == 'code' and self.p.annot.get(a + 4, (None,))[0] == 'data':
                t = self.p.annot[a + 4][1]
                tocs[t] = tocs.get(t, 0) + 1
        self.toc = max(tocs, key=tocs.get)
        self.traps = {}
        for i, name in enumerate(self.p.imports):
            code = TRAP_CODE + 4 * i
            uc.mem_write(TRAP_BASE + 8 * i, struct.pack('>II', code, self.toc))
            self.traps[code] = name
        for a, (k, v) in self.p.annot.items():
            if k == 'imp':
                uc.mem_write(a, struct.pack('>I', TRAP_BASE + 8 * v))
        self.handlers = {
            'sqrt': lambda: self._fret(math.sqrt(self.freg(1)) if self.freg(1) >= 0 else float('nan')),
            'log10': lambda: self._fret(math.log10(self.freg(1)) if self.freg(1) > 0 else float('-inf')),
            'pow': lambda: self._fret(self._pow(self.freg(1), self.freg(2))),
            'NewPtr': lambda: self.alloc(self.reg(3)),
            'NewPtrClear': lambda: self.alloc(self.reg(3)),
            'NewPtrSys': lambda: self.alloc(self.reg(3)),
            'NewPtrSysClear': lambda: self.alloc(self.reg(3)),
            'DisposePtr': lambda: 0,
            'BlockMove': self._blockmove,
            'BlockMoveData': self._blockmove,
            'MemError': lambda: 0,
            'TickCount': lambda: 0,
        }
        if extra_imports:
            self.handlers.update(extra_imports)
        self.unhandled = set()
        uc.hook_add(UC_HOOK_CODE, self._hook, begin=TRAP_CODE, end=TRAP_CODE + 0xFFFF)

    # -------------------------------------------------------------- traps
    @staticmethod
    def _pow(x, y):
        try:
            return math.pow(x, y)
        except (ValueError, OverflowError):
            return float('nan') if x < 0 else float('inf')

    def _fret(self, v):
        self.uc.reg_write(UC_PPC_REG_FPR1, struct.unpack('>Q', struct.pack('>d', v))[0])
        return None

    def _blockmove(self):
        src, dst, n = self.reg(3), self.reg(4), self.reg(5)
        if n:
            self.uc.mem_write(dst, bytes(self.uc.mem_read(src, n)))
        return 0

    def _hook(self, uc, addr, size, ud):
        name = self.traps.get(addr)
        if name is None:
            return
        h = self.handlers.get(name)
        if h is None:
            if name not in self.unhandled:
                self.unhandled.add(name)
                print('# emu: unhandled import %s (returning 0)' % name, file=sys.stderr)
            r = 0
        else:
            r = h()
        if r is not None:
            uc.reg_write(UC_PPC_REG_3, r & 0xFFFFFFFF)

    # ------------------------------------------------------------- memory
    def alloc(self, n, data=None):
        a = (self.heap + 15) & ~15
        self.heap = a + max(n, 1)
        if self.heap > HEAP_BASE + HEAP_SIZE:
            raise MemoryError('emulated heap exhausted')
        self.uc.mem_write(a, bytes(data) if data is not None else bytes(n))
        return a

    def reset_heap(self):
        self.heap = HEAP_BASE + 0x1000

    def read(self, a, n):
        return bytes(self.uc.mem_read(a, n))

    def write(self, a, b):
        self.uc.mem_write(a, bytes(b))

    def r32(self, a):
        return struct.unpack('>I', self.read(a, 4))[0]

    def w32(self, a, v):
        self.write(a, struct.pack('>I', v & 0xFFFFFFFF))

    def reg(self, n):
        return self.uc.reg_read(UC_PPC_REG_0 + n)

    def freg(self, n):
        return struct.unpack('>d', struct.pack('>Q', self.uc.reg_read(UC_PPC_REG_FPR0 + n)))[0]

    # ----------------------------------------------------------- addresses
    def ghidra_to_emu(self, a):
        """translate a Ghidra code address (0x10000000-based) to the emulated one"""
        return a - GHIDRA_CODE + self.code_base

    def data_addr(self, ghidra_addr):
        """translate a Ghidra data address to the emulated one.  Ghidra's PEF
        loader places the data section right after the code section."""
        code = self.p.secs[self.p.code_idx[0]]
        data = self.p.secs[self.p.data_idx[0]]
        gdata = GHIDRA_CODE + code['total']
        return ghidra_addr - gdata + data['base']

    def func(self, name_or_addr):
        if isinstance(name_or_addr, int):
            a = name_or_addr
            if a >= GHIDRA_CODE:
                a = self.ghidra_to_emu(a)
        else:
            a = None
            for addr, n in self.p.syms.items():
                if n in ('.' + name_or_addr, name_or_addr) or n.lstrip('.').startswith(name_or_addr + '__'):
                    a = addr
                    break
            if a is None:
                raise KeyError(name_or_addr)
        while self.r32(a) == 0:  # traceback padding before the prologue
            a += 4
        return a

    # --------------------------------------------------------------- calls
    def call_words(self, fn, words, fargs=(), max_insns=0):
        """words: the parameter area (r3..r10 then stack); fargs: f1..f13"""
        uc = self.uc
        sp = STACK_TOP
        for i, v in enumerate(words):
            if i < 8:
                uc.reg_write(UC_PPC_REG_3 + i, v & 0xFFFFFFFF)
            uc.mem_write(sp + 24 + 4 * i, struct.pack('>I', v & 0xFFFFFFFF))
        for i, v in enumerate(fargs):
            uc.reg_write(UC_PPC_REG_FPR1 + i, struct.unpack('>Q', struct.pack('>d', v))[0])
        uc.reg_write(UC_PPC_REG_1, sp)
        uc.reg_write(UC_PPC_REG_2, self.toc)
        uc.reg_write(UC_PPC_REG_LR, RET_ADDR)
        uc.reg_write(UC_PPC_REG_MSR, uc.reg_read(UC_PPC_REG_MSR) | 0x2000)
        addr = self.func(fn)
        try:
            uc.emu_start(addr, RET_ADDR, count=max_insns)
        except UcError as e:
            raise RuntimeError('%s: emulation error %s at pc=%x' % (fn, e, uc.reg_read(UC_PPC_REG_PC)))
        return self.reg(3), self.freg(1)

    def call(self, fn, args=(), fargs=()):
        return self.call_words(fn, list(args), fargs)

    # -------------------------------------------------------------- structs
    def pack_doubles(self, vals):
        return b''.join(struct.pack('>d', v) for v in vals)

    def unpack_doubles(self, a, n):
        return list(struct.unpack('>%dd' % n, self.read(a, 8 * n)))


if __name__ == '__main__':
    lib = sys.argv[1] if len(sys.argv) > 1 else '../../../DS_Dual4_extracted/MFSLib2891'
    m = Emu(lib)
    # smoke test: FUN_1002b820 is abs(int)
    print('abs(-5) =', m.call(0x1002B820, [(-5) & 0xFFFFFFFF])[0])
    print('toc = %x, code base = %x, imports = %d' % (m.toc, m.code_base, len(m.p.imports)))
