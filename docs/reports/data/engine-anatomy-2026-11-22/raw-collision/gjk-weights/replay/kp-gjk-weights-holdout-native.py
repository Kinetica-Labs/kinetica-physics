import csv,difflib,hashlib,json,os,subprocess,tarfile
from pathlib import Path
w=Path(__file__).resolve().parent;root=w.parent.parent;out=w/'results'; h=w/'holdout';h.mkdir(exist_ok=True)
sdk=root/'accepted-src';arc=root/'accepted-build/src/libkinetica-physics.a'
cc=Path(r'C:\Users\sshuser\CODE\kinetica-physics\tools\llvm-mingw-20261117-ucrt-x86_64\bin\clang++.exe')
os.environ['PATH']=str(cc.parent)+os.pathsep+os.environ['PATH'];flags=['-std=c++17','-O3','-DNDEBUG','-ffp-contract=off','-I'+str(sdk/'include')]
rel='validation/studies/anatomy/primitive_feature_pairs.cpp'
base=(sdk/rel).read_text(encoding='utf8')
probe=base
for x,y in [('(second ? .73 : -.18)','(second ? -.52 : .37)'),('frames{{{}, {{7.25, -3.75, 12.5}, Quaternion{.82, .19, -.31, .42}.unit()}}}','frames{{{{-4, 2, 3}, Quaternion{1, -2, 3, 5}.unit()}, {{2, 7, -1}, Quaternion{3, 1, 4, -2}.unit()}}}'),('k ? "rotated_translated" : "identity"','k ? "holdout_B" : "holdout_A"')]:
 assert probe.count(x)==1,x
 probe=probe.replace(x,y)
(out/'holdout.patch').write_text(''.join(difflib.unified_diff(base.splitlines(True),probe.splitlines(True),'a/'+rel,'b/'+rel)),encoding='utf8')
p=h/'probe.cpp';p.write_text(probe,encoding='utf8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
record={}
for name,objs in [('base',[w/'base/Gjk.obj']),('area',[w/'area/Gjk.obj']),('select',[w/'kp-gjk-weights-selector.cpp',w/'select/base.obj',w/'select/area.obj'])]:
 exe=h/(name+'.exe');subprocess.run([cc,*flags,*objs,p,arc,'-o',exe],check=True)
 table=out/('holdout-'+name+'.csv')
 with table.open('wb') as f,(h/(name+'.txt')).open('wb') as tr:
    rc=subprocess.run([exe],stdout=f,stderr=tr).returncode
 with table.open() as f:rows=list(csv.DictReader(f))
 assert len(rows)==1440 and all(r['oracle_ok']=='1' for r in rows)
 record[name]={'exit':rc,'passes':sum(r['pass']=='1' for r in rows),'csv_sha256':sha(table),'exe_sha256':sha(exe)}
 print(name,record[name],flush=True)
manifest=json.loads((out/'manifest.json').read_text());manifest['holdout']={'probe_sha256':sha(p),'results':record}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
with tarfile.open(w/'weights-result.tgz','w:gz') as tar:
 for p in sorted(out.iterdir()):
    if p.is_file():tar.add(p,arcname=p.name)
