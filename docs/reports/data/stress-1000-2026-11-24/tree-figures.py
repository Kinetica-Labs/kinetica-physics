"""Сводка парных запусков из неизменённого архива Windows."""
import csv
import io
import json
import math
import statistics as stats
import tarfile
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path('docs/reports')
data = root/'data/stress-1000-2026-11-24'
scenes = {'pyramid1000':'Обрушение', 'rain1000':'Падение решётки'}
totals, blocks = [], []
def write(path, rows):
    with path.open('w',newline='') as f:
        out=csv.DictWriter(f,fieldnames=rows[0].keys(),lineterminator='\n')
        out.writeheader()
        out.writerows(rows)
def q99(samples):
    return sorted(samples)[math.ceil(.99*len(samples))-1]
with tarfile.open(data/'tree-study-windows.tar.gz') as ts:
    def get(name):
        return ts.extractfile('./'+name).read().decode('utf-8-sig')
    manifest=json.loads(get('manifest.json'))
    for scene in scenes:
        pooled={'base':[],'trial':[]}
        pairs=[]
        for block in range(1,7):
            means={}
            for key in pooled:
                rows=list(csv.DictReader(io.StringIO(get(f'{scene}-{block}-{key}.csv'))))
                if [int(row['frame']) for row in rows]!=list(range(1,121)) or any(row['final_hash']!=manifest['hashes'][scene] for row in rows):
                    raise ValueError(f'incomplete series {scene} {key} {block}')
                frames=[float(row['step_us'])/1000 for row in rows]
                if not all(math.isfinite(t) and t>0 for t in frames):
                    raise ValueError('invalid time')
                means[key]=stats.mean(frames)
                pooled[key].extend(frames)
            gain=100*(1-means['trial']/means['base'])
            pairs.append((means['base'],means['trial'],gain))
            blocks.append(dict(scene=scene,block=block,first='base' if block%2 else 'trial',base_ms=means['base'],trial_ms=means['trial'],reduction_pct=gain))
        wins=sum(g>0 for _,_,g in pairs)
        n=sum(g!=0 for _,_,g in pairs)
        p=wins/n if n else .5
        z=stats.NormalDist().inv_cdf(.975)
        centre=(p+z*z/(2*n))/(1+z*z/n) if n else .5
        half=z*math.sqrt((p*(1-p)+z*z/(4*n))/n)/(1+z*z/n) if n else .5
        prob=min(1,2*sum(math.comb(n,k) for k in range(min(wins,n-wins)+1))/2**n)
        a,b,gains=zip(*pairs)
        totals.append(dict(scene=scene,blocks=len(pairs),base_ms=stats.median(a),trial_ms=stats.median(b),reduction_percent=stats.median(gains),minimum=min(gains),maximum=max(gains),wins=wins,sign_p=prob,wilson_lo=max(0,centre-half),wilson_hi=min(1,centre+half),base_p99_ms=q99(pooled['base']),trial_p99_ms=q99(pooled['trial'])))
write(data/'tree-speed.csv',totals)
write(data/'tree-time-pairs.csv',blocks)
fig, (left, right)=plt.subplots(1,2,figsize=(11,3.6),constrained_layout=True)
for i,(scene,title) in enumerate(scenes.items()):
    subset=[b for b in blocks if b['scene']==scene]
    color=['#116d5e','#365d9d'][i]
    for entry in subset:
        left.plot([i-.18,i+.18],[entry['base_ms'],entry['trial_ms']],color=color,alpha=.6,marker='o',markersize=4,linewidth=1.4)
    right.scatter([b['block'] for b in subset],[b['reduction_pct'] for b in subset],marker=['o','s'][i],s=48,label=title,color=color)
    right.axhline(totals[i]['reduction_percent'],color=color,alpha=.5,linestyle='--')
left.set_xticks([-.18,.18,.82,1.18],['Контроль\nобрушение','Новый\nобрушение','Контроль\nрешётка','Новый\nрешётка'])
left.set_ylabel('Среднее время 120 кадров, мс')
right.set_ylabel('Сокращение среднего времени, %')
right.set_xticks(range(1,7))
right.set_xlabel('Номер парного блока')
right.axhline(0,linewidth=1,color='#666666')
right.legend(loc='upper left',fontsize=9,frameon=False)
left.set_title('Два измерения на одном ядре')
right.set_title('Каждая точка — отдельная пара')
for ax in (left,right):
    ax.grid(axis='y',alpha=.18)
    ax.spines[['top','right']].set_visible(False)
fig.savefig(root/'figures/stress-tree-paired.png',dpi=160)
