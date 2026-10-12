#!/usr/bin/env python3
"""
Differential-test harness: executes the ORIGINAL PowerPC code of the Konica
Minolta driver (MFSBaseLib2891, a CFM/PEF shared library) in the Unicorn CPU
emulator and drives it with scenario.txt.  SCSI commands issued through the
library's MFSScsiExecCommand hook are routed to the same protocol simulator
the C port uses (build/libsimx.dylib), and every exchange is printed in the
same canonical format as port_runner.c, so the two traces can be diffed.

usage: ppc_runner.py MFSBaseLib2891 scenario.txt [DS_Dual4.BIN]
requires: pip install unicorn   (and pef.py from docs/re-tools)
"""
import ctypes
import os
import struct
import sys

from unicorn import Uc, UC_ARCH_PPC, UC_MODE_PPC32, UC_MODE_BIG_ENDIAN, UC_HOOK_CODE, UcError
from unicorn.ppc_const import *

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'docs', 're-tools'))
from pef import PEF  # noqa: E402

TRAP_BASE = 0x0F000000          # fake transition vectors + trap code
TRAP_CODE = TRAP_BASE + 0x10000
RET_ADDR = TRAP_BASE + 0x1F000  # return sentinel
STACK_TOP = 0x7F0F0000
HEAP_BASE = 0x60000000
HEAP_SIZE = 0x08000000


def fnv(b):
    h = 1469598103934665603
    for x in b:
        h ^= x
        h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return h


class Machine:
    def __init__(self, lib, fw):
        self.p = PEF(lib)
        self.p.find_functions()
        self.uc = Uc(UC_ARCH_PPC, UC_MODE_PPC32 | UC_MODE_BIG_ENDIAN)
        try:
            self.uc.ctl_set_cpu_model(UC_CPU_PPC32_7400_V2_9)
        except Exception:
            pass
        for s in self.p.secs:
            if s['kind'] in (0, 1, 2, 3, 6):
                base = s['base'] & ~0xFFF
                size = ((s['base'] + max(s['total'], 1) - base) + 0xFFF) & ~0xFFF
                self.uc.mem_map(base, size)
                self.uc.mem_write(s['base'], bytes(s['body']))
        self.uc.mem_map(TRAP_BASE, 0x20000)
        self.uc.mem_write(TRAP_CODE, struct.pack('>I', 0x4E800020) * ((0x20000 - 0x10000) // 4))
        self.uc.mem_map(STACK_TOP - 0x100000, 0x110000)
        self.uc.mem_map(HEAP_BASE, HEAP_SIZE)
        self.heap = HEAP_BASE + 0x1000
        # TOC = most common data pointer stored right after a code pointer (TVectors)
        tocs = {}
        for a, (k, v) in self.p.annot.items():
            if k == 'code' and self.p.annot.get(a + 4, (None,))[0] == 'data':
                t = self.p.annot[a + 4][1]
                tocs[t] = tocs.get(t, 0) + 1
        self.toc = max(tocs, key=tocs.get)
        # imports -> fake transition vectors pointing at trap code
        self.traps = {}
        nimp = len(self.p.imports)
        for i, name in enumerate(self.p.imports):
            self._make_trap(i, name)
        for a, (k, v) in self.p.annot.items():
            if k == 'imp':
                self.uc.mem_write(a, struct.pack('>I', TRAP_BASE + 8 * v))
        self.extra = {}
        self.extra_tv = {}
        for j, name in enumerate(['MFSScsiExecCommand', 'MFSScsiGetScannerInfo']):
            self._make_trap(nimp + j, name)
            self.extra_tv[name] = TRAP_BASE + 8 * (nimp + j)
        self.uc.hook_add(UC_HOOK_CODE, self._hook, begin=TRAP_CODE, end=TRAP_CODE + 0xFFFF)
        self.fw_path = fw
        self.fw_data = b''
        self.fw_pos = 0
        # GetFWFileRefID locates the Preferences folder; short-circuit it to "found"
        fid = self.func('GetFWFileRefID')
        self.uc.hook_add(UC_HOOK_CODE, self._override_zero, begin=fid, end=fid)
        self.ticks = 1000
        self.unknown = set()
        # simulator
        self.sim_lib = ctypes.CDLL(os.path.join(HERE, '..', '..', 'build', 'libsimx.dylib'))
        self.sim_lib.simx_open.restype = ctypes.c_void_p
        self.sim_lib.simx_open.argtypes = [ctypes.c_char_p]
        self.sim_lib.simx_exec.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p,
                                           ctypes.c_size_t, ctypes.POINTER(ctypes.c_uint8)]
        self.sim = self.sim_lib.simx_open(fw.encode() if fw else None)

    def _make_trap(self, i, name):
        code = TRAP_CODE + 4 * i
        self.uc.mem_write(TRAP_BASE + 8 * i, struct.pack('>II', code, self.toc if hasattr(self, 'toc') else 0))
        self.traps[code] = name

    # ---------------------------------------------------------- memory
    def r32(self, a):
        return struct.unpack('>I', self.uc.mem_read(a, 4))[0]

    def w32(self, a, v):
        self.uc.mem_write(a, struct.pack('>I', v & 0xFFFFFFFF))

    def reg(self, n):
        return self.uc.reg_read(UC_PPC_REG_0 + n)

    def alloc(self, n, clear=True):
        a = self.heap
        self.heap = (self.heap + n + 15) & ~15
        if self.heap > HEAP_BASE + HEAP_SIZE:
            raise MemoryError('emulated heap exhausted')
        if clear and n:
            self.uc.mem_write(a, bytes(n))
        return a

    def pstr(self, a):
        n = self.uc.mem_read(a, 1)[0]
        return bytes(self.uc.mem_read(a + 1, n)).decode('mac_roman')

    # ---------------------------------------------------------- traps
    def _hook(self, uc, addr, size, ud):
        name = self.traps.get(addr)
        if name is None:
            return
        a = [self.reg(3 + i) for i in range(8)]
        r = 0
        if name in ('NewPtr', 'NewPtrClear', 'NewPtrSys', 'NewPtrSysClear'):
            r = self.alloc(a[0])
        elif name in ('DisposePtr', 'DisposeHandle', 'HLock', 'HUnlock'):
            r = 0
        elif name in ('BlockMove', 'BlockMoveData'):
            if a[2]:
                uc.mem_write(a[1], bytes(uc.mem_read(a[0], a[2])))
        elif name == 'TickCount':
            self.ticks += 7
            r = self.ticks
        elif name == 'Delay':
            self.ticks += a[0]
            if a[1]:
                self.w32(a[1], self.ticks)
        elif name == 'Gestalt':
            if a[1]:
                self.w32(a[1], 0)
        elif name == 'GetSharedLibrary':
            self.w32(a[3], 1)
        elif name == 'FindSymbol':
            sym = self.pstr(a[1])
            if sym in self.extra_tv:
                self.w32(a[2], self.extra_tv[sym])
            else:
                r = -2804
        elif name == 'FSMakeFSSpec' and self.fw_path:
            r = 0
        elif name == 'FSpOpenDF' and self.fw_path:
            self.fw_data = open(self.fw_path, 'rb').read()
            self.fw_pos = 0
            uc.mem_write(a[2], struct.pack('>h', 7))
        elif name == 'SetFPos':
            self.fw_pos = a[2]
        elif name == 'GetEOF':
            self.w32(a[1], len(self.fw_data))
        elif name == 'FSRead':
            n = self.r32(a[1])
            chunk = self.fw_data[self.fw_pos:self.fw_pos + n]
            uc.mem_write(a[2], chunk)
            self.w32(a[1], len(chunk))
            self.fw_pos += len(chunk)
        elif name == 'FSClose':
            r = 0
        elif name in ('FindFolder', 'FSMakeFSSpec', 'FSpOpenDF', 'FSpCreate', 'PBGetCatInfoSync',
                      'DirCreate', 'FSpDirCreate', 'HOpenDF', 'PBHOpenDFSync'):
            r = -43
        elif name == 'MFSScsiExecCommand':
            r = self._scsi(a[1])
        elif name == 'MFSScsiGetScannerInfo':
            self.w32(a[1], 0xC)
            self.w32(a[1] + 4, 4)
            self.w32(a[1] + 8, 0)
        elif name == 'MemError':
            r = 0
        else:
            if name not in self.unknown:
                self.unknown.add(name)
                print('# unhandled import', name, file=sys.stderr)
        uc.reg_write(UC_PPC_REG_3, r & 0xFFFFFFFF)

    def _override_zero(self, uc, addr, size, ud):
        uc.reg_write(UC_PPC_REG_3, 0)
        uc.reg_write(UC_PPC_REG_PC, uc.reg_read(UC_PPC_REG_LR))

    def _scsi(self, info):
        cdbp, cdbl, data, dlen, d = (self.r32(info + o) for o in (0, 4, 8, 12, 16))
        stp = self.r32(info + 0x1C)
        cdb = bytes(self.uc.mem_read(cdbp, 10))
        buf = (ctypes.c_uint8 * max(dlen, 1))()
        if d == 2 and dlen:
            ctypes.memmove(buf, bytes(self.uc.mem_read(data, dlen)), dlen)
        st = ctypes.c_uint8(0xFF)
        self.sim_lib.simx_exec(self.sim, cdb, d, ctypes.addressof(buf), dlen, ctypes.byref(st))
        line = 'CMD ' + ' '.join('%02x' % x for x in cdb) + ' dir=%d len=%d' % (d, dlen)
        if d == 2:
            line += ' out=' + bytes(buf)[:dlen].hex()
        elif d == 1:
            payload = bytes(buf)[:dlen]
            if st.value == 0 and dlen:
                self.uc.mem_write(data, payload)
            line += ' in#%016x' % fnv(payload)
        print(line + ' st=%d' % st.value)
        self.uc.mem_write(stp, bytes([st.value]))
        return 0

    # ---------------------------------------------------------- calls
    def func(self, name):
        cb = self.p.secs[self.p.code_idx[0]]
        for a, n in self.p.syms.items():
            if n in ('.' + name, name):
                while self.r32(a) == 0:
                    a += 4
                return a
        raise KeyError(name)

    def call(self, name, *args, fargs=()):
        uc = self.uc
        for i, v in enumerate(args):
            uc.reg_write(UC_PPC_REG_3 + i, v & 0xFFFFFFFF)
        for i, v in enumerate(fargs):
            uc.reg_write(UC_PPC_REG_FPR1 + i, struct.unpack('>Q', struct.pack('>d', v))[0])
        uc.reg_write(UC_PPC_REG_1, STACK_TOP)
        uc.reg_write(UC_PPC_REG_2, self.toc)
        uc.reg_write(UC_PPC_REG_LR, RET_ADDR)
        uc.reg_write(UC_PPC_REG_MSR, uc.reg_read(UC_PPC_REG_MSR) | 0x2000)  # FP available
        try:
            uc.emu_start(self.func(name), RET_ADDR)
        except UcError as e:
            pc = uc.reg_read(UC_PPC_REG_PC)
            raise RuntimeError('%s: emulation error %s at pc=%x' % (name, e, pc))
        return uc.reg_read(UC_PPC_REG_3)


def sp_bytes(v, flags):
    """the 0x60 byte AV scan parameter block, big-endian as on the PowerPC"""
    b = bytearray(0x60)
    (ulx, uly, w, l, px, ln, mode, bits, _, er, eg, eb, xr, yr, focus, ml, mscan, msamp, aps, final, nonstd) = v[:21]
    struct.pack_into('>HHHHHH', b, 0, ulx, uly, w, l, px, ln)
    b[0x0c] = mode
    b[0x0e] = bits
    b[0x12] = 1      # halftone
    b[0x13] = 0xFF   # highlight
    struct.pack_into('>HHHH', b, 0x18, flags, er, eg, eb)
    struct.pack_into('>HH', b, 0x20, xr, yr)
    struct.pack_into('>hH', b, 0x2A, focus, ml)
    b[0x2E] = mscan
    b[0x2F] = msamp
    struct.pack_into('>i', b, 0x30, aps)
    struct.pack_into('>i', b, 0x40, final)
    struct.pack_into('>i', b, 0x48, nonstd)
    b[0x4C] = 100
    return bytes(b)


def main():
    lib, scen = sys.argv[1], sys.argv[2]
    fw = sys.argv[3] if len(sys.argv) > 3 else None
    m = Machine(lib, fw)
    if m.call('AVILibInit') != 0:
        print('# AVILibInit failed', file=sys.stderr)
    H = 0
    img = m.alloc(1 << 22)
    spbuf = m.alloc(0x60)
    tmp = m.alloc(0x100)
    for line in open(scen):
        t = line.split()
        if not t or t[0].startswith('#'):
            continue
        c = t[0]
        if c == 'INIT':
            print('== INIT')
            m.call('InitializeDriver')
            m.call('SetSpeedMode', 1)
            print('ret %d' % m.call('FindScanner', H))
            r = m.call('CheckScannerState', H, 2, tmp)
            print('ret %d state %d' % (r, m.uc.mem_read(tmp, 1)[0]))
            print('ret %d' % m.call('GetScannerAbilityEx', H, tmp + 0x10))
        elif c == 'CAL':
            print('== CAL')
            print('ret %d' % m.call('DoCalibration', H))
        elif c == 'SP':
            name = t[1]
            nums = [int(x, 0) for x in t[2:]]
            flags = nums[8]
            m.uc.mem_write(spbuf, sp_bytes(nums, flags))
            print('== SP %s' % name)
            ok = m.call('SetScanParameter', H, spbuf)
            print('ret %d' % ok)
            if not ok:
                continue
            print('ret %d' % m.call('StartScan', H))
            # bytes per output line as computed by the library ([aa9e])
            px, bits = nums[4], nums[7]
            lb = px * bits // 8
            total, chunk = nums[21], nums[22]
            done = 0
            while done < total:
                n = min(chunk, total - done)
                r = m.call('ReadScan', H, 0, img, n * lb)
                data = bytes(m.uc.mem_read(img, n * lb))
                if bits in (48, 16):  # samples are 16-bit big-endian in PPC memory -> compare host LE
                    data = b''.join(data[i + 1:i + 2] + data[i:i + 1] for i in range(0, len(data), 2))
                print('IMG %d %d #%016x' % (r, n * lb, fnv(data)))
                done += n
            print('ret %d' % m.call('StopScan', H))
        elif c == 'DARK':
            e = [float(x) for x in t[1:4]]
            print('== DARK')
            print('ret %d' % m.call('DarkCalibrationData', H, fargs=e))
        elif c == 'FOCUS':
            pos = int(t[1])
            print('== FOCUS %d' % pos)
            print('ret %d' % m.call('SetOpticalControl', H, pos & 0xFFFF))
        elif c == 'FOCUSSCAN':
            x, y, w, cnt = (int(v) for v in t[1:5])
            pos = [int(v) for v in t[5:]]
            print('== FOCUSSCAN')
            v = [x, y, w, 1, w, 1, 4, 48, 0, 152, 152, 152, 3200, 3200, pos[0], 1, cnt, 0, 0, 0, 0]
            b = bytearray(sp_bytes(v, (0x9048 & 0xF7FF) | 0x10A))
            struct.pack_into('>i', b, 0x38, 1)
            m.uc.mem_write(spbuf, bytes(b))
            print('ret %d' % m.call('SetScanParameter', H, spbuf))
            print('ret %d' % m.call('StartScan', H))
            lb = w * 6
            for p in pos:
                print('ret %d' % m.call('SetOpticalControl', H, p & 0xFFFF))
                for _ in range(cnt):
                    r = m.call('ReadScan', H, 0, img, lb)
                    data = bytes(m.uc.mem_read(img, lb))
                    data = b''.join(data[i + 1:i + 2] + data[i:i + 1] for i in range(0, len(data), 2))
                    print('IMG %d %d #%016x' % (r, lb, fnv(data)))
            print('ret %d' % m.call('StopScan', H))
        elif c == 'HOLDER':
            n = int(t[1])
            print('== HOLDER %d' % n)
            print('ret %d' % m.call('SendHolderControl', H, n))
        elif c == 'FEED':
            print('== FEED')
            print('ret %d' % m.call('DoPaperFeed', H))
        elif c == 'SETSTATE':
            n = int(t[1])
            print('== SETSTATE %d' % n)
            m.uc.mem_write(tmp, bytes([n]))
            print('ret %d' % m.call('CheckScannerState', H, 1, tmp))
        elif c == 'INITSCANNER':
            print('== INITSCANNER')
            print('ret %d' % m.call('InitializeScanner', H))
        elif c == 'FWDL':
            print('== FWDL')
            name = m.alloc(16)
            m.uc.mem_write(name, b'\x0cDS_Dual4.BIN')
            print('ret %d' % (m.call('InternalDownLoadFirmware', H, name) if fw else 0))
        elif c == 'RESET':
            print('== RESET')
            print('ret %d' % m.call('ResetToInitialState', H, 0))
        sys.stdout.flush()


if __name__ == '__main__':
    main()
