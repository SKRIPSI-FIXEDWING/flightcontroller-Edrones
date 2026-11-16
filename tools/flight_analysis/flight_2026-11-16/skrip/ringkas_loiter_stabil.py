"""Metrik loiter fase stabil (putaran ke-2 dst) per run AUTO dari log SD. R lewat env R (default 100).
Pakai: R=100 python3 ringkas_loiter_stabil.py LOG.CSV [...]"""
import sys,os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
import numpy as np
from sdlog_io import read_sdlog
from analyze_fuzzy_sdlog import loiter_turn_index
def runs(mask):
    r=[];s=None
    for i,f in enumerate(mask):
        if f and s is None:s=i
        if not f and s is not None:r.append((s,i));s=None
    if s is not None:r.append((s,len(mask)))
    return r
print("file,run,R,fuzzy,steady_s,putaran,mean_m,rms_m,maxabs_m,pct_out10,bebas_s,bias_bebas_m,sd_bebas_m,rms_bebas_m,n_pulsa_steady,per_mean,per_min,per_max,gs_mean")
for fn in sys.argv[1:]:
    L=read_sdlog(fn);c=L.col;t=c['time_boot_ms']/1000
    wps=[s for s in L.snapshots if s.waypoints];lw=max(w['seq'] for w in wps[-1].waypoints)
    eff=L.effective_at(float(c['time_boot_ms'][-1]))
    dist=[e.t_ms/1000 for e in L.events_of('DIST_ON')]
    auto=np.array([m=='AUTO' for m in c['mode']])&(c['armed']>0)
    for k,(a,b) in enumerate(runs(auto)):
        idx=np.arange(a,b);idx=idx[c['wp_next'][idx]==lw]
        if len(idx)<100:continue
        turn=loiter_turn_index(c['lat_e7'][idx],c['lon_e7'][idx])
        st=idx[turn>=1]
        if len(st)<20:print(f"{os.path.basename(fn)},{k},,,{len(st)*0.05:.0f},,,,,,,,,,,,,,");continue
        Rr=float(os.environ.get('R','100'))
        e=c['wp_dist_m'][st]-Rr;ts=t[st]
        free=np.ones(len(st),bool)
        for d in dist: free&=~((ts>=d)&(ts<=d+15))
        ef=e[free];dt=np.median(np.diff(ts))
        np_st=sum(1 for d in dist if ts[0]<=d<=ts[-1])
        per=c['l1_period_s'][st]
        ang=np.unwrap(np.arctan2((c['lat_e7'][idx]-c['lat_e7'][idx].mean()),(c['lon_e7'][idx]-c['lon_e7'][idx].mean())))
        put=abs(ang[-1]-ang[0])/(2*np.pi)
        print(f"{os.path.basename(fn)},{k},{Rr:.0f},{int(c['fuzzy_on'][st].max())},{len(st)*dt:.0f},{put:.1f},{e.mean():.1f},{np.sqrt((e**2).mean()):.1f},{abs(e).max():.1f},{100*(abs(e)>10).mean():.0f},{free.sum()*dt:.0f},{ef.mean() if len(ef) else np.nan:.1f},{ef.std() if len(ef) else np.nan:.1f},{np.sqrt((ef**2).mean()) if len(ef) else np.nan:.1f},{np_st},{per.mean():.2f},{per.min():.2f},{per.max():.2f},{c['gs_mps'][st].mean():.1f}")
