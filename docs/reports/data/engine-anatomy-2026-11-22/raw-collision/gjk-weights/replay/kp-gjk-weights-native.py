import csv,hashlib,json,os,shutil,subprocess,tarfile
from pathlib import Path
from collections import Counter
work=Path(__file__).resolve().parent
root=work.parent.parent
raw=work/'results'; raw.mkdir(exist_ok=True)
sdk=root/'accepted-src'; archive=root/'accepted-build/src/libkinetica-physics.a'
cc=Path(r'C:\Users\sshuser\CODE\kinetica-physics\tools\llvm-mingw-20261117-ucrt-x86_64\bin\clang++.exe')
os.environ['PATH']=str(cc.parent)+os.pathsep+os.environ['PATH']
gjk=Path('src/collision/narrowphase/Gjk.cpp'); probe=Path('validation/studies/anatomy/primitive_feature_pairs.cpp')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
flags=['-std=c++17','-O3','-DNDEBUG','-ffp-contract=off','-I'+str(sdk/'include')]
digits=work/'digits.cpp'; digits.write_text('#include <limits>\n#include <cstdio>\nint main(){std::printf("%d %d %zu",std::numeric_limits<double>::digits,std::numeric_limits<long double>::digits,sizeof(long double));}\n')
subprocess.run([cc,*flags,digits,'-o',work/'digits.exe'],check=True)
d,w,sz=map(int,subprocess.check_output([work/'digits.exe']).split())
assert w>d
info={'compiler':subprocess.check_output([cc,'--version']).decode().splitlines()[0],'target':subprocess.check_output([cc,'-dumpmachine']).decode().strip(),'flags':flags,'double_digits':d,'long_double_digits':w,'long_double_size':sz,'gjk_sha256':sha(sdk/gjk),'probe_sha256':sha(sdk/probe),'accepted_library_sha256':sha(archive)}
summary={}; keys0=set()
for name in ('base','gram','area'):
    dr=work/name; target=dr/gjk
    target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(sdk/gjk,target)
    if name!='base':
        patch=work/('kp-gjk-weights-'+name+'.patch')
        subprocess.run(['git.exe','apply','--whitespace=nowarn',patch],cwd=dr,check=True)
        shutil.copyfile(patch,raw/(name+'.patch'))
    obj=dr/'Gjk.obj'; exe=dr/'primitive.exe'
    subprocess.run([cc,*flags,'-c',target,'-o',obj],check=True)
    subprocess.run([cc,*flags,obj,sdk/probe,archive,'-o',exe],check=True)
    with (raw/(name+'.csv')).open('wb') as fp,(raw/(name+'.txt')).open('wb') as err:
        rc=subprocess.run([exe],stdout=fp,stderr=err).returncode
    assert rc in (0,1)
    with (raw/(name+'.csv')).open() as fp: rows=list(csv.DictReader(fp))
    keys={tuple(r[f] for f in ('feature_a','feature_b','frame','swap','gap_m')) for r in rows}
    assert len(rows)==len(keys)==1440 and all(r['oracle_ok']=='1' for r in rows)
    if name=='base':keys0=keys
    else:assert keys==keys0
    summary[name]={'exit':rc,'passes':sum(r['pass']=='1' for r in rows),'csv_sha256':sha(raw/(name+'.csv')),'source_sha256':sha(target),'object_sha256':sha(obj),'exe_sha256':sha(exe),'components':dict(Counter(r['closest_ok']+r['presence_ok']+r['witness_ok'] for r in rows))}
    if name=='base':assert summary[name]['passes']==1412
    print(name,summary[name],flush=True)
(raw/'manifest.json').write_text(json.dumps(dict(environment=info,results=summary),indent=2)+'\n')
with tarfile.open(work/'weights-result.tgz','w:gz') as f:
    for p in sorted(raw.iterdir()):
        if p.is_file():f.add(p,arcname=p.name)
