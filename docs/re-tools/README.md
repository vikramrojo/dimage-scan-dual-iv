# Reverse-engineering tools

These are the tools used to recover the scanner protocol from the original
PowerPC driver.  They are not needed to build or use the driver.

| file | purpose |
|---|---|
| `pef.py` | Loader/disassembler for CFM "Joy!peff" PowerPC fragments: section unpacking (pattern-initialized data), relocation, imports/exports, function names from traceback tables, TOC-relative annotation. `python3 pef.py MFSBaseLib2891 out.s` |
| `funcstarts.py` | Emits every traceback-named function start for Ghidra. |
| `dis_macho.py` | Annotated disassembly of the Mach-O PPC USB bundle (`MFSIOUsb2891`). |
| `ghidra/FixToc.java` | Pre-script: NOPs the `lwz r2,20(r1)` TOC reloads after cross-fragment calls so Ghidra keeps r2 constant and resolves globals. |
| `ghidra/CreateFuncs.java` | Post-script: creates functions at the traceback-derived starts (vtable-only methods are otherwise missed). |
| `ghidra/FixSigs.java` | Fixes the signatures of `AVAllocateMem`, `SwapWORD`, `SwapDWORD` (the decompiler otherwise merges r3:r4). |
| `ghidra/DumpDecomp.java` | Decompiles every function to one C file. |
| `ghidra/DumpMem.java` | Dumps constants (follows TOC pointers, prints doubles/floats). |
| `../../tests/diff/ppc_runner.py` | Runs the original `MFSBaseLib2891` code under the Unicorn PowerPC emulator for differential testing. |

Typical Ghidra run (Ghidra 12, PowerPC:BE:32:default; on Apple silicon build
the decompiler with `make ghidra_opt` and copy it to `os/mac_arm_64/decompile`):

    python3 funcstarts.py MFSBaseLib2891 > funcs.txt
    analyzeHeadless proj p -import MFSBaseLib2891 -processor PowerPC:BE:32:default \
        -scriptPath ghidra -preScript FixToc.java \
        -postScript CreateFuncs.java funcs.txt -postScript FixSigs.java \
        -postScript DumpDecomp.java MFSBaseLib.c
