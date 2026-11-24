from pathlib import Path
import sys,subprocess,os,hashlib,json,struct
lab=Path(sys.argv[1]); here=Path(__file__).resolve().parent
cc=Path(r'C:\Users\sshuser\CODE\kinetica-physics\tools\llvm-mingw-20261117-ucrt-x86_64\bin\clang++.exe')
os.environ['PATH']=str(cc.parent)+os.pathsep+os.environ['PATH']
def capture(build):
    exe=here/('matrix-'+build+'.exe')
    subprocess.run([cc,'-O3','-DNDEBUG','-std=c++17','-ffp-contract=off','-I'+str(lab/'accepted-src/include'),here/'matrix.cpp',lab/build/'src/libkinetica-physics.a','-o',exe],check=True)
    return subprocess.check_output([exe])
a,b=map(capture,['accepted-build','normal-candidate-build'])
def counts(s):
    at=0; rows=0; entries=0
    for i in range(1024):
        nr=struct.unpack_from('<I',s,at)[0];at+=4;rows+=nr
        for j in range(nr):
            ne=struct.unpack_from('<I',s,at)[0]; at+=4+ne*12;entries+=ne
    assert at==len(s);return rows,entries
result={'seed':19730819,'cases':1024,'rows_entries':counts(a),'bytes':len(a),'byte_equal':a==b,
        'sha256_base':hashlib.sha256(a).hexdigest(),'sha256_candidate':hashlib.sha256(b).hexdigest(),'sha256_driver':hashlib.sha256((here/'matrix.cpp').read_bytes()).hexdigest()}
(here/'matrix-parity.json').write_text(json.dumps(result,indent=2))
print(result)
if a!=b:sys.exit(1)
