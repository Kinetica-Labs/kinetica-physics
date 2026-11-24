import csv,hashlib,json,os,shutil,subprocess,tarfile
from pathlib import Path
w=Path(__file__).resolve().parent
out=w/'results';sdk=w.parent.parent/'accepted-src'
arc=w.parent.parent/'accepted-build/src/libkinetica-physics.a'
cc=Path(r'C:\Users\sshuser\CODE\kinetica-physics\tools\llvm-mingw-20261117-ucrt-x86_64\bin\clang++.exe')
os.environ['PATH']=str(cc.parent)+os.pathsep+os.environ['PATH'];flags=['-std=c++17','-O3','-DNDEBUG','-ffp-contract=off','-I'+str(sdk/'include')]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
dr=w/'select';dr.mkdir(exist_ok=True)
objects=[]
for name,alias,flag in [('base','originalProjection','originalShrank'),('area','areaProjection','areaShrank')]:
    src=(w/name/'src/collision/narrowphase/Gjk.cpp').read_text(encoding='utf8')
    code='    const auto* cylinderA ='
    assert src.count(code)==1
    src=src.replace(code, '    extern bool '+flag+';\n    '+flag+' = result.coresOverlap;\n'+code)
    p=dr/(name+'.cpp');p.write_text(src,encoding='utf8')
    obj=dr/(name+'.obj');subprocess.run([cc,*flags,'-DclosestPoints='+alias,'-c',p,'-o',obj],check=True);objects.append(obj)
probe=sdk/'validation/studies/anatomy/primitive_feature_pairs.cpp'
src=probe.read_text(encoding='utf8')
line='    const auto closest = closestPoints('
assert src.count(line)==1
probe2=dr/'probe.cpp';probe2.write_text(src.replace(line,'    std::fprintf(stderr,"case,%s,%s,%s,%d,%.9g\\n",a.name,b.name,frame,swap,gap);\n'+line),encoding='utf8')
selector=w/'kp-gjk-weights-selector.cpp';shutil.copyfile(selector,out/'selector.cpp')
exe=dr/'primitive-select.exe'
subprocess.run([cc,*flags,selector,*objects,probe2,arc,'-o',exe],check=True)
with (out/'select.csv').open('wb') as f,(out/'select-trace.txt').open('wb') as tr:rc=subprocess.run([exe],stdout=f,stderr=tr).returncode
with (out/'select.csv').open() as f:rows=list(csv.DictReader(f))
assert rc in (0,1) and len(rows)==1440
print('select',sum(r['pass']=='1' for r in rows),flush=True)
manifest=json.loads((out/'manifest.json').read_text());manifest['results']['selector']={'passes':sum(r['pass']=='1' for r in rows),'exit':rc,'source_sha256':sha(selector),'exe_sha256':sha(exe),'csv_sha256':sha(out/'select.csv'),'trace_sha256':sha(out/'select-trace.txt')}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
with tarfile.open(w/'weights-result.tgz','w:gz') as tar:
 for p in sorted(out.iterdir()):
    if p.is_file():tar.add(p,arcname=p.name)
