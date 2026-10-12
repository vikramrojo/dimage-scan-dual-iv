#!/usr/bin/env python3
"""Minimal PEF (CFM PowerPC) loader + annotated disassembler.

Usage: pef.py <file> [outfile]
Produces annotated disassembly: function names from traceback tables / exports,
TOC (r2) loads resolved to imports, strings and code/data targets.
"""
import struct, sys, re, capstone

KIND = {0:'code',1:'data',2:'pidata',3:'const',4:'loader',5:'debug',6:'execdata',7:'exception',8:'traceback'}

def u32(b,o): return struct.unpack_from('>I',b,o)[0]
def s32(b,o): return struct.unpack_from('>i',b,o)[0]
def u16(b,o): return struct.unpack_from('>H',b,o)[0]

def unpack_pidata(p, size):
    out = bytearray(); i = 0
    def arg():
        nonlocal i
        v = 0
        while True:
            c = p[i]; i += 1
            v = (v << 7) | (c & 0x7f)
            if not c & 0x80: return v
    while i < len(p):
        b = p[i]; i += 1
        op, cnt = b >> 5, b & 0x1f
        if cnt == 0: cnt = arg()
        if op == 0: out += bytes(cnt)
        elif op == 1: out += p[i:i+cnt]; i += cnt
        elif op == 2:
            rep = arg(); blk = p[i:i+cnt]; i += cnt; out += blk*(rep+1)
        elif op == 3:
            common = cnt; custom = arg(); rep = arg()
            c = p[i:i+common]; i += common
            for _ in range(rep):
                out += c + p[i:i+custom]; i += custom
            out += c
        elif op == 4:
            common = cnt; custom = arg(); rep = arg()
            for _ in range(rep):
                out += bytes(common) + p[i:i+custom]; i += custom
            out += bytes(common)
        else: raise ValueError('bad pidata op %d' % op)
    out += bytes(max(0, size-len(out)))
    return bytes(out)

class PEF:
    def __init__(self, path):
        d = self.raw = open(path,'rb').read()
        assert d[:8] == b'Joy!peff'
        nsec = u16(d,32)
        self.secs = []
        for k in range(nsec):
            o = 40 + 28*k
            name_off, defaddr, total, unpacked, packed, coff = struct.unpack_from('>iIIIII', d, o)
            kind = d[o+24]
            raw = d[coff:coff+packed]
            if kind == 2: body = unpack_pidata(raw, total)
            else: body = raw + bytes(max(0, total-len(raw)))
            self.secs.append(dict(kind=kind, total=total, raw=raw, body=bytearray(body), off=coff))
        # assign virtual bases: code at 0x10000000-ish, others spaced
        base = 0x100000
        for s in self.secs:
            s['base'] = base; base += (s['total'] + 0xfffff) & ~0xffff
        self.code_idx = [i for i,s in enumerate(self.secs) if s['kind'] in (0,6)]
        self.data_idx = [i for i,s in enumerate(self.secs) if s['kind'] in (1,2)]
        self.annot = {}   # absolute addr -> description (relocated words)
        self.syms = {}    # absolute addr -> name
        self.parse_loader()

    def parse_loader(self):
        ls = [s for s in self.secs if s['kind'] == 4][0]['raw']
        (mainS, mainO, initS, initO, termS, termO, nlib, nimp, nrel, relInstr,
         strOff, hashOff, hashPow, nexp) = struct.unpack_from('>iIiIiIIIIIIIII', ls, 0)
        cstr = lambda o: ls[strOff+o:ls.index(b'\0', strOff+o)].decode('mac_roman')
        self.libs = []; self.imports = []
        for k in range(nlib):
            no, ov, cv, cnt, first = struct.unpack_from('>IIIII', ls, 56+24*k)
            self.libs.append((cstr(no), first, cnt))
        symtab = 56 + 24*nlib
        for k in range(nimp):
            v = u32(ls, symtab+4*k)
            self.imports.append(cstr(v & 0xffffff))
        self.implib = {}
        for name, first, cnt in self.libs:
            for k in range(first, first+cnt): self.implib[k] = name
        # exports
        keytab = hashOff + 4*(1 << hashPow)
        exptab = keytab + 4*nexp
        for k in range(nexp):
            klen = u32(ls, keytab+4*k) >> 16
            can, val, sidx = struct.unpack_from('>IIh', ls, exptab+10*k)
            name = ls[strOff+(can & 0xffffff):strOff+(can & 0xffffff)+klen].decode('mac_roman')
            if sidx >= 0:
                self.syms[self.secs[sidx]['base'] + val] = 'export:' + name
        # relocations
        rh = symtab + 4*nimp
        for k in range(nrel):
            sidx, _, cnt, first = struct.unpack_from('>HHII', ls, rh+12*k)
            instrs = [u16(ls, relInstr+first+2*j) for j in range(cnt)]
            self.relocate(sidx, instrs)

    def relocate(self, sidx, ins):
        sec = self.secs[sidx]; body = sec['body']
        pos = 0
        C = self.secs[self.code_idx[0]]['base'] if self.code_idx else 0
        D = self.secs[self.data_idx[0]]['base'] if self.data_idx else 0
        imp = 0
        def add(kind, val):
            nonlocal pos
            w = u32(body, pos)
            if kind == 'imp':
                self.annot[sec['base']+pos] = ('imp', val)
            else:
                self.annot[sec['base']+pos] = (kind, w + val)
                struct.pack_into('>I', body, pos, (w + val) & 0xffffffff)
            pos += 4
        i = 0
        while i < len(ins):
            x = ins[i]; i += 1
            if x >> 14 == 0:
                skip, cnt = (x >> 6) & 0xff, x & 0x3f
                pos += 4*skip
                for _ in range(cnt): add('data', D)
            elif x >> 13 == 2:
                sub, run = (x >> 9) & 0xf, (x & 0x1ff) + 1
                for _ in range(run):
                    if sub == 0: add('code', C)
                    elif sub == 1: add('data', D)
                    elif sub == 2: add('code', C); add('data', D); pos += 4
                    elif sub == 3: add('code', C); add('data', D)
                    elif sub == 4: add('data', D); pos += 4
                    elif sub == 5: add('imp', imp); imp += 1
                    else: raise ValueError(sub)
            elif x >> 13 == 3:
                sub, idx = (x >> 9) & 0xf, x & 0x1ff
                if sub == 0: imp = idx; add('imp', imp); imp += 1
                elif sub == 1: C = self.secs[idx]['base']
                elif sub == 2: D = self.secs[idx]['base']
                elif sub == 3: add('data', self.secs[idx]['base'])
            elif x >> 12 == 8:
                pos += (x & 0xfff) + 1
            elif x >> 12 == 9:
                blk, rep = ((x >> 8) & 0xf) + 1, (x & 0xff) + 1
                # repeat the preceding blk instructions rep times
                seg = ins[i-1-blk:i-1]
                ins = ins[:i] + seg*rep + ins[i:]
            elif x >> 10 == 0x28:
                pos = ((x & 0x3ff) << 16) | ins[i]; i += 1
            elif x >> 10 == 0x29:
                imp = ((x & 0x3ff) << 16) | ins[i]; i += 1
                add('imp', imp); imp += 1
            elif x >> 10 == 0x2c:
                blk = ((x >> 6) & 0xf) + 1; rep = ((x & 0x3f) << 16) | ins[i]; i += 1
                seg = ins[i-2-blk:i-2]
                ins = ins[:i] + seg*rep + ins[i:]
            elif x >> 10 == 0x2d:
                sub = (x >> 6) & 0xf; idx = ((x & 0x3f) << 16) | ins[i]; i += 1
                if sub == 0: add('data', self.secs[idx]['base'])
                elif sub == 1: C = self.secs[idx]['base']
                elif sub == 2: D = self.secs[idx]['base']
            else:
                raise ValueError('reloc op %04x' % x)

    # ---- helpers
    def sec_of(self, a):
        for s in self.secs:
            if s['base'] <= a < s['base'] + s['total']: return s
    def word(self, a):
        s = self.sec_of(a); return u32(s['body'], a - s['base']) if s else None
    def cstring(self, a, maxlen=120):
        s = self.sec_of(a)
        if not s: return None
        b = s['body'][a-s['base']:a-s['base']+maxlen]
        z = b.find(b'\0')
        if z < 2:
            # maybe pascal string
            return None
        t = b[:z]
        if all(32 <= c < 127 or c in (9,10,13) for c in t): return t.decode()
        return None

    def describe(self, a):
        """describe what a TOC slot / data pointer refers to"""
        an = self.annot.get(a)
        if an is None: return None
        k, v = an
        if k == 'imp': return 'IMP:' + self.imports[v]
        if v in self.syms: return self.syms[v]
        s = self.cstring(v)
        if s is not None: return 'str:' + repr(s)
        if k == 'code': return 'code:%x' % v
        # data: tvector?
        an2 = self.annot.get(v)
        if an2 and an2[0] == 'code':
            n = self.syms.get(an2[1])
            return 'tv:' + (n or 'code:%x' % an2[1])
        if an2 and an2[0] == 'imp': return '&IMP:' + self.imports[an2[1]]
        return 'data:%x' % v

    def find_functions(self):
        """traceback tables: 0x00000000 followed by version 0 tb; name after optional fields"""
        for ci in self.code_idx:
            s = self.secs[ci]; b = s['body']; n = len(b)
            starts = [0]
            o = 0
            while o < n - 8:
                if u32(b,o) == 0 and b[o+4] == 0 and o >= 4 and u32(b, o-4) != 0:
                    # traceback table
                    try:
                        name, end = self.parse_tb(b, o+4)
                    except Exception:
                        o += 4; continue
                    if name is not None:
                        # function start: search back to previous end
                        st = starts[-1]
                        self.syms.setdefault(s['base']+st, name)
                        starts.append(end)
                        o = end; continue
                o += 4

    def parse_tb(self, b, o):
        # struct tbtable_short: version, lang, flags1(globallink,is_eprol,has_tboff,int_proc,has_ctl,tocless,fp_present,log_abort),
        # flags2(int_hndl,name_present,uses_alloca,cl_dis_inv(3),saves_cr,saves_lr), flags3(stores_bc, fixedparms?), ...
        if b[o] != 0: return None, 0
        f1 = b[o+2]; f2 = b[o+3]
        fixedparms = b[o+6]; flags5 = b[o+7]
        floatparms = flags5 >> 1
        has_tboff = f1 & 0x20; int_proc = f1 & 0x10; has_ctl = f1 & 0x08
        name_present = f2 & 0x40; uses_alloca = f2 & 0x20
        p = o + 8
        if fixedparms or floatparms: p += 4
        tboff = None
        if has_tboff: tboff = u32(b,p); p += 4
        if int_proc: p += 4
        if has_ctl:
            nctl = u32(b,p); p += 4 + 4*nctl
        if not name_present: return None, 0
        ln = u16(b,p); p += 2
        if not 0 < ln < 256: return None, 0
        nm = bytes(b[p:p+ln])
        if not all(32 < c < 127 for c in nm): return None, 0
        p += ln
        if uses_alloca: p += 1
        p = (p + 3) & ~3
        return nm.decode(), p

    def disasm(self, out):
        self.find_functions()
        md = capstone.Cs(capstone.CS_ARCH_PPC, capstone.CS_MODE_32 | capstone.CS_MODE_BIG_ENDIAN)
        # TOC base: all TVectors (code,data) pairs share one toc; pick the most common data value following a code reloc
        tocs = {}
        for a,(k,v) in self.annot.items():
            if k == 'code' and self.annot.get(a+4, (None,))[0] == 'data':
                t = self.annot[a+4][1]; tocs[t] = tocs.get(t,0)+1
                n = self.syms.get(v)
                if n and n.startswith('export:'): pass
        self.toc = max(tocs, key=tocs.get) if tocs else 0
        # name functions referenced by tvectors via exports
        for a,n in list(self.syms.items()):
            if n.startswith('export:') and self.annot.get(a,(None,))[0] == 'code':
                tgt = self.annot[a][1]
                self.syms.setdefault(tgt, n[7:])
        print('; TOC = %x' % self.toc, file=out)
        for i,(n,f,c) in enumerate(self.libs): print('; lib', n, f, c, file=out)
        for ci in self.code_idx:
            s = self.secs[ci]
            prev = None
            body = bytes(s['body'])
            def gen():
                for o in range(0, len(body) - 3, 4):
                    r = list(md.disasm(body[o:o+4], s['base'] + o))
                    if r: yield r[0]
                    else:
                        a = s['base'] + o
                        if a in self.syms: print('\n%s:' % self.syms[a], file=out)
                        print('%8x: %08x  .long' % (a, u32(body, o)), file=out)
            for ins in gen():
                if ins.address in self.syms: print('\n%s:' % self.syms[ins.address], file=out)
                c = ''
                ops = ins.op_str
                m = re.search(r'(-?0x[0-9a-f]+|-?\d+)\(r2\)$', ops)
                if m:
                    off = int(m.group(1), 0); slot = self.toc + off
                    c = self.describe(slot) or ('toc+%x' % off)
                if ins.mnemonic in ('bl','b') or ins.mnemonic.startswith('b') and ops.startswith('0x'):
                    try:
                        t = int(ops.split(',')[-1].strip(), 16)
                        n = self.syms.get(t)
                        if n: c = n
                        else:
                            # glue: lwz r12, X(r2)
                            w = self.word(t)
                            if w and (w >> 16) == 0x8182:
                                off = struct.unpack('>h', struct.pack('>H', w & 0xffff))[0]
                                c = 'glue->' + (self.describe(self.toc + off) or '?')
                    except ValueError: pass
                print('%8x: %08x  %-8s %-28s %s' % (ins.address, u32(ins.bytes,0), ins.mnemonic, ops, c), file=out)

if __name__ == '__main__':
    p = PEF(sys.argv[1])
    out = open(sys.argv[2], 'w') if len(sys.argv) > 2 else sys.stdout
    for i,s in enumerate(p.secs): print('; sec %d %s size %x base %x' % (i, KIND.get(s['kind']), s['total'], s['base']), file=out)
    p.disasm(out)
    # dump data strings
