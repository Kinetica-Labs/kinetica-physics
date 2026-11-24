"""Пересчитать время из каталога распакованной серии qualified-time.ps1."""
import csv
import json
import math
import statistics
import sys
from pathlib import Path

def read(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))

def summarize(folder, output):
    runs = read(folder / 'order.csv')
    manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8-sig'))
    rows = []
    for scene in ('pyramid1000','rain1000','rest55'):
        pairs = []
        pooled = {'on': [], 'off': []}
        for i in range(1,7):
            jobs = [r for r in runs if r['scene']==scene and int(r['pair'])==i]
            expected = ['off','on'] if i%2 else ['on','off']
            assert [(int(r['slot']), r['mode']) for r in jobs]==list(enumerate(expected, 1))
            means = {}
            for job in jobs:
                mode = job['mode']
                series = read(folder / f'{scene}-{i}-{mode}.csv')
                assert [int(x['frame']) for x in series]==list(range(1,121))
                assert {x['final_hash'] for x in series}=={manifest['hashes'][f'{scene}-{mode}']}
                times = [float(x['step_us'])/1000 for x in series]
                assert all(math.isfinite(t) and t>0 for t in times)
                pooled[mode].extend(times)
                means[mode] = statistics.mean(times)
            pairs.append((means['off'], means['on']))
        changes = [100*(on/off-1) for off,on in pairs]
        n = sum(x!=0 for x in changes)
        slower = sum(x>0 for x in changes)
        z = statistics.NormalDist().inv_cdf(.975)
        p = slower/n
        centre = (p+z*z/(2*n))/(1+z*z/n)
        radius = z*math.sqrt(p*(1-p)/n + z*z/(4*n*n))/(1+z*z/n)
        sign_p = min(1,2*sum(math.comb(n,k) for k in range(min(slower,n-slower)+1))/2**n)
        rows.append(dict(scene=scene,pairs=len(pairs),off_ms=statistics.median(p[0] for p in pairs),
                         on_ms=statistics.median(p[1] for p in pairs),paired_delta_ms=statistics.median(on-off for off,on in pairs),
                         paired_percent=statistics.median(changes),min_percent=min(changes),max_percent=max(changes),slower=slower,
                         off_p99_ms=sorted(pooled['off'])[math.ceil(.99*len(pooled['off']))-1],
                         on_p99_ms=sorted(pooled['on'])[math.ceil(.99*len(pooled['on']))-1],
                         wilson_lo=max(0,centre-radius),wilson_hi=min(1,centre+radius),sign_p=sign_p))
    with output.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
if __name__=='__main__':
    summarize(Path(sys.argv[1]),Path(sys.argv[2]))
