import sys,subprocess,re,capstone
f=sys.argv[1]; d=open(f,'rb').read()
syms={}
for l in subprocess.run(['nm','-n',f],capture_output=True,text=True).stdout.splitlines():
    p=l.split()
    if len(p)==3: syms[int(p[0],16)]=p[2]
# stub map: indirect symbols
out=subprocess.run(['otool','-Iv',f],capture_output=True,text=True).stdout
stubs={}
for l in out.splitlines():
    m=re.match(r'\s*0x([0-9a-f]+)\s+\d+\s+(\S+)',l)
    if m: stubs[int(m.group(1),16)]=m.group(2)
txt_off=3976; addr=0xf88; size=0x1e14+0x264
md=capstone.Cs(capstone.CS_ARCH_PPC,capstone.CS_MODE_32|capstone.CS_MODE_BIG_ENDIAN)
for i in md.disasm(d[txt_off:txt_off+size],addr):
    if i.address in syms: print(f"\n{syms[i.address]}:")
    c=''
    if i.mnemonic in('bl','b'):
        t=int(i.op_str,16)
        c=syms.get(t,'') or stubs.get(t,'')
        if not c:
            # picsymbol stub: find stub index
            if 0x2d9c<=t<0x2d9c+0x264: c='stub#%d'%((t-0x2d9c)//36)
    print(f"{i.address:5x}: {i.mnemonic:8s} {i.op_str:30s} {c}")
