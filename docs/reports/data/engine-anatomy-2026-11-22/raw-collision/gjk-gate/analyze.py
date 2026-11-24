import csv,collections,json,sys
from pathlib import Path
here=Path(__file__).resolve().parent; w=here/'results'
original=Path('docs/reports/data/engine-anatomy-2026-11-22/raw-collision/gjk-weights')
manifest=json.loads((w/'manifest.json').read_text())
def read(p):return list(csv.DictReader(p.open()))
out=[];paired=[]
for name,prefix in [('primary',''),('holdout','holdout-')]:
 rs=[read(f) for f in [original/(prefix+'base.csv'),original/(prefix+'select.csv'),w/(name+'.csv')]]
 gate=manifest['series'][name];g=gate['counters'];f=manifest['series'][name+'-forced-cost']['counters']
 assert all(len(t)==1440 for t in rs)
 status=collections.Counter(); delta=collections.Counter(); changecases=[]
 checks=['oracle_ok','closest_ok','presence_ok','witness_ok']
 for r0,r1,r2 in zip(*rs):
  key=['feature_a','feature_b','swap','frame','gap_m'];assert all(r0[x]==r1[x]==r2[x] for x in key)
  status[r0['pass']+r1['pass']+r2['pass']]+=1
  for chk in checks:
   if r1[chk]!=r2[chk]:delta[chk+':'+r1[chk]+'>'+r2[chk]]+=1
  if r1['pass']!=r2['pass'] or any(r1[c]!=r2[c] for c in checks):
   paired.append([name,*[r0[k] for k in key],r0['pass'],r1['pass'],r2['pass'],','.join(k for k in checks if r1[k]!=r2[k])])
 stock=g['first_descent_supports']+g['first_pointcore_supports']
 gcert=g['first_certificate_supports']+g['second_certificate_supports']
 repeated=g['second_descent_supports']+g['second_pointcore_supports']
 fc=f['first_descent_supports']+f['second_descent_supports']+f['first_pointcore_supports']+f['second_pointcore_supports']
 oldChecks=2*(f['original_resolved']+f['alternate_resolved'])
 out.append(dict(probe=name,queries=gate['queries'],passes=list(map(lambda rows:sum(r['pass']=='1' for r in rows),rs)),patterns_stock_fullselect_gated=dict(status),predicate_changes_from_fullselect=dict(delta),retries=g['retry'],selected_area=g['alternate'],baseline_support_calls=stock,repeat_support_calls=repeated,certificate_support_calls=gcert,gate_total_support_calls=stock+repeated+gcert,always_two_descent_support_calls=fc,old_fullselect_extra_support_calls=oldChecks,old_fullselect_total_support_calls=fc+oldChecks,bounding_radius_calls=g['first_bound_calls']+g['second_bound_calls']))
r,s=map(read,[w/'contract.csv',w/'contract-stock.csv']);assert len(r)==len(s)
for a,b in zip(r,s):
 assert all(a[k]==b[k] for k in ('case','shape_a','shape_b','frame','swap','step','margin_m'))
ret=dict(primitive=out,contract=dict(rows=len(r),gate_passes=sum(row['pass']=='1' for row in r),stock_passes=sum(row['pass']=='1' for row in s),new_fails=sum(a['pass']=='0' and b['pass']=='1' for a,b in zip(r,s)),gates=manifest['series']['contract']['counters']),guard_tables_equal=(w/'guards-stock.csv').read_bytes()==(w/'guards-gate.csv').read_bytes(),guards=manifest['series']['guards-gate'])
(w/'summary.json').write_text(json.dumps(ret,indent=2));print(json.dumps(ret,indent=2))
with (w/'differences.csv').open('w') as f:
 z=csv.writer(f);z.writerow(['probe','feature_a','feature_b','swap','frame','gap_m','stock_pass','fullselect_pass','gate_pass','changed_gates']);z.writerows(paired)
