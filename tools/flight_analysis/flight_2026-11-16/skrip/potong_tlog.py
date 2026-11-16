"""potong_tlog.py IN OUT t0 t1 : simpan rekaman tlog dalam [t0,t1] (detik sejak awal tlog).
Rekaman param/misi/home di luar jendela tetap disimpan (dipakai skrip analisis)."""
import sys,struct
KEEP={22,39,44,47,73,242}  # PARAM_VALUE, MISSION_ITEM(_INT), MISSION_COUNT, MISSION_ACK, HOME_POSITION
src,dst,t0,t1=sys.argv[1],sys.argv[2],float(sys.argv[3]),float(sys.argv[4])
b=open(src,'rb').read();out=bytearray();i=0;n=len(b);T0=None;kept=0
while i+10<n:
    ts=struct.unpack('>Q',b[i:i+8])[0];m=b[i+8]
    if m==0xFE: L=b[i+9];mid=b[i+13];j=i+8+6+L+2
    elif m==0xFD: L=b[i+9];inc=b[i+10];mid=int.from_bytes(b[i+15:i+18],'little');j=i+8+10+L+2+(13 if inc&1 else 0)
    else: i+=1;continue
    if T0 is None:T0=ts
    t=(ts-T0)/1e6
    if t0<=t<=t1 or mid in KEEP: out+=b[i:j];kept+=1
    i=j
open(dst,'wb').write(out);print(dst,kept,'paket')
