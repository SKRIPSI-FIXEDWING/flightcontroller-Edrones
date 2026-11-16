"""potong.py IN OUT t0_s t1_s : simpan baris data dalam [t0,t1], event waktu-kejadian di luar jendela dibuang."""
import sys
src,dst,t0,t1=sys.argv[1],sys.argv[2],float(sys.argv[3])*1000,float(sys.argv[4])*1000
DROP={"DIST_ON","DIST_OFF","MODE","ARM","DISARM","WP_REACHED"}
hdr=False
with open(src,errors='ignore') as f,open(dst,'w') as g:
    for line in f:
        if line.startswith('#EVENT,'):
            p=line.split(',')
            if p[2].strip() in DROP and not(t0<=int(p[1])<=t1):continue
            g.write(line);continue
        if line.startswith('#'):g.write(line);continue
        if not hdr:g.write(line);hdr=True;continue
        try:t=int(line.split(',',1)[0])
        except ValueError:continue
        if t0<=t<=t1:g.write(line)
