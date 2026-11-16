"""Respons bertanda tiap pulsa gangguan di loiter stabil, dikelompokkan. Argumen: "GRUP|LOG.CSV|R" ... (grup L1 dan FZ wajib untuk uji Mann-Whitney)."""
import sys,os,statistics as stt
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import numpy as np
from sdlog_io import read_sdlog
from analyze_fuzzy_sdlog import loiter_turn_index
from scipy.stats import mannwhitneyu
def runs(mask):
    r=[];s=None
    for i,f in enumerate(mask):
        if f and s is None:s=i
        if not f and s is not None:r.append((s,i));s=None
    if s is not None:r.append((s,len(mask)))
    return r
groups={}
for spec in sys.argv[1:]:
    g,fn,R=spec.split('|');R=float(R)
    L=read_sdlog(fn);c=L.col;t=c['time_boot_ms']/1000
    lw=max(w['seq'] for w in [s for s in L.snapshots if s.waypoints][-1].waypoints)
    dist=[e.t_ms/1000 for e in L.events_of('DIST_ON')]
    auto=np.array([m=='AUTO' for m in c['mode']])&(c['armed']>0)
    for a,b in runs(auto):
        idx=np.arange(a,b);idx=idx[c['wp_next'][idx]==lw]
        if len(idx)<100:continue
        turn=loiter_turn_index(c['lat_e7'][idx],c['lon_e7'][idx]);st=idx[turn>=1]
        if len(st)<20:continue
        ts=t[st];e=c['wp_dist_m'][st]-R
        ds=[d for d in dist if ts[0]+2<=d<=ts[-1]]
        for i,d in enumerate(ds):
            nxt=ds[i+1] if i+1<len(ds) else ts[-1]
            pre=e[(ts>=d-2)&(ts<d)]
            if len(pre)<5:continue
            e0=np.median(pre);w=(ts>=d)&(ts<=min(d+15,nxt));ew=e[w];tw=ts[w]
            if len(ew)<20:continue
            k=np.argmax(ew);dpos=ew[k]-e0;tpk=tw[k]-d
            after=ew[k:];under=after.min()-e0
            # pulih: sesudah puncak kembali ke |e-e0|<=3 m bertahan 2 s
            rec=np.nan
            ok=np.abs(ew-e0)<=3
            for j in range(k,len(ew)):
                jj=(tw>=tw[j])&(tw<=tw[j]+2)
                if ok[jj].all() and tw[j]+2<=tw[-1]:rec=tw[j]-d;break
            groups.setdefault(g,[]).append(dict(f=os.path.basename(fn),t=d,e0=e0,dpos=dpos,tpk=tpk,under=under,rec=rec,gap=nxt-d,emax=np.abs(ew).max(),gs=c['gs_mps'][st][w].mean()))
for g,v in groups.items():
    m=lambda k:stt.median([x[k] for x in v if np.isfinite(x[k])])
    nrec=sum(np.isfinite(x['rec']) for x in v)
    print(f"{g:10s} n={len(v)} e0={m('e0'):+.1f} geser_keluar={m('dpos'):.1f} t_puncak={m('tpk'):.1f} undershoot={m('under'):+.1f} |e|maks={m('emax'):.1f} pulih={m('rec'):.1f} (terukur {nrec}/{len(v)}) jarak={m('gap'):.1f} gs={m('gs'):.1f}")
for f in sorted(set(x['f'] for v in groups.values() for x in v)):
    for g,v in groups.items():
        w=[x for x in v if x['f']==f]
        if w: print(f"   {g:10s} {f:26s} n={len(w)} geser={stt.median([x['dpos'] for x in w]):.1f} e0={stt.median([x['e0'] for x in w]):+.1f} |e|maks={stt.median([x['emax'] for x in w]):.1f}")
a,b=groups['L1'],groups['FZ']
for k in('dpos','emax','under'):
    print(k,'MWU p=%.3f'%mannwhitneyu([x[k] for x in a],[x[k] for x in b]).pvalue)
