#!/usr/bin/env python3
"""Print 'address name' lines for every function named by a traceback table in
a PEF fragment, in Ghidra's address layout (code section at 0x10000000).
Feed the output to ghidra/CreateFuncs.java."""
import struct, sys
from pef import PEF
p = PEF(sys.argv[1]); p.find_functions()
s = p.secs[p.code_idx[0]]; cb = s['base']; b = s['body']
for a, n in sorted(p.syms.items()):
    if cb <= a < cb + s['total'] and not n.startswith('export:'):
        o = a - cb
        while o + 4 <= len(b) and struct.unpack_from('>I', b, o)[0] == 0:
            o += 4
        print('%x %s' % (o + 0x10000000, n.lstrip('.')))
