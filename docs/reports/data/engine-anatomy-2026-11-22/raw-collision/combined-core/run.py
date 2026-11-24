"""On Windows, preserve accepted build exactly and apply the supplied experiment in a copy."""
import csv
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

lab=Path(sys.argv[1]).resolve()
out=Path(__file__).resolve().parent
base=lab/'cap-manifold-src'; base_build=lab/'cap-manifold-build'
src=lab/'combined-core-src'; build=lab/'combined-core-build'
refdir=lab/'combined-core-reference-probes'
import shlex
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
def inventory(d):
    return {p.relative_to(d).as_posix():digest(p) for p in d.rglob('*') if p.is_file()}
def run(args,log,**kwargs):
    with (out/log).open('wb') as f:
        code=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT,**kwargs).returncode
    if code:
        print((out/log).read_text(encoding='utf-8',errors='replace')[-3500:])
        raise RuntimeError((code,args))
def configuration(d):
    cache=(d/'CMakeCache.txt').read_text(encoding='utf-8')
    result = {}
    for line in cache.splitlines():
        if line and not line.startswith(('#','//')) and '=' in line and ':' in line.partition('=')[0]:
            key, _, value = line.partition('=')
            result[key.split(':',1)[0]] = value
    return result

try:
    details=json.loads((out/'snapshot.json').read_text())
    worktree={r['path']:r['worktree_sha256'] for r in csv.DictReader((out/'worktree-hashes.csv').open())}
    assert digest(out/'worktree.tgz')==details['worktree_tgz']
    if not src.exists():
        src.mkdir()
        with tarfile.open(out/'worktree.tgz') as bundle: bundle.extractall(src,filter='data')
    now=inventory(src); baseline=inventory(base)
    if now!=worktree or set(now)!=set(baseline):raise RuntimeError('source inventory mismatch')
    difference={k for k in now if now[k]!=baseline[k]}
    if difference!=set(details['solver_files'])|{details['script_only']}:raise RuntimeError(('delta against cap',difference))
    with (out/'seven-source-hashes.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(('path','working_sha256','cap_reference_sha256','compiled_snapshot_sha256'))
        for k in sorted(details['solver_files']+details['geometry_files']):w.writerow((k,worktree[k],baseline[k],now[k]))
    cap_core=base_build/'src/libkinetica-physics.a';cap_scenes=base_build/'examples/libkinetica-scenes.a'
    if digest(cap_core)!=details['cap_library']:raise RuntimeError('cap-manifold reference archive differs from final manifest')
    c=configuration(base_build)
    env=dict(os.environ,KP_THREADS='1',PATH=str(Path(c['CMAKE_CXX_COMPILER']).parent)+os.pathsep+os.environ['PATH'])
    keys=['CMAKE_CXX_COMPILER','CMAKE_MAKE_PROGRAM','CMAKE_BUILD_TYPE','CMAKE_CXX_FLAGS',
          'CMAKE_CXX_FLAGS_RELWITHDEBINFO','CMAKE_EXE_LINKER_FLAGS','CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO','KP_VIEWER']
    run(['cmake','-S',src,'-B',build,'-G',c['CMAKE_GENERATOR'],*[f'-D{k}={c[k]}' for k in keys]],'configure.log',env=env)
    cfg=configuration(build)
    for k in keys:
        if c[k].replace('\\','/')!=cfg[k].replace('\\','/'): raise RuntimeError(f'{k} changed')
    group=['study-grip-limit','study-oblique-grip','study-groove-friction','study-state-hash']
    suite=['unit-tests','FloatingBox','GearTrain','study-slip-to-roll','study-rest-overlay','study-playback-clock',
           'study-hull-events','study-hull-motion','study-collision-contract','study-bucket-pairs']
    run(['cmake','--build',build,'--target',*group,'compare-ours',*suite,'--parallel','4'],'build.log',env=env)
    ctest=['ctest','--test-dir',str(build)]
    # Win32 CreateProcess cannot execute the project's two POSIX .sh checks directly.
    # Change only the build-generated test command to include the installed Git Bash.
    # The project source, archive paths, shell scripts, checks and their results remain intact.
    bash=Path(r'C:\Program Files\Git\bin\bash.exe')
    if not bash.is_file():raise RuntimeError('Git Bash unavailable for registered shell tests')
    generated=build/'tests'/'CTestTestfile.cmake'
    rawfile=generated.read_text(encoding='utf-8')
    adapted=rawfile
    for registered in ['no-libm','no-libm-scenes']:
        prefix=f'add_test({registered} "'
        if adapted.count(prefix)!=1: raise RuntimeError(f'unrecognized Windows CTest command: {registered}')
        adapted=adapted.replace(prefix,f'add_test({registered} "{bash.as_posix()}" "')
    (out/'ctest-generated-original.txt').write_text(rawfile)
    (out/'ctest-generated-win-launcher.txt').write_text(adapted)
    generated.write_text(adapted,encoding='utf-8')
    llvm_dir=Path(c['CMAKE_CXX_COMPILER']).parent
    bin_alias=build/'ctest-nm'; bin_alias.mkdir(exist_ok=True)
    shutil.copy2(llvm_dir/'llvm-nm.exe',bin_alias/'nm.exe')
    env['PATH']=str(bin_alias)+os.pathsep+env['PATH']
    if (out/'ctest.log').is_file() and not (out/'ctest-original.log').exists():
        shutil.copyfile(out/'ctest.log',out/'ctest-original.log')
    run(ctest+['--show-only=json-v1'],'ctest-list.json',env=env)
    tests=json.loads((out/'ctest-list.json').read_text(encoding='utf-8'))
    if len(tests['tests'])!=22: raise RuntimeError(f"Missing tests: {len(tests['tests'])}")
    run(ctest+['--output-on-failure','-j2'],'ctest.log',env=env)
    nm=Path(c['CMAKE_CXX_COMPILER']).parent/'llvm-nm.exe'
    banned=re.compile(r'^(?:sin|cos|tan|asin|acos|atan|atan2|sinh|cosh|tanh|exp|exp2|log|log2|log10|pow|cbrt|hypot)f?l?$')
    with (out/'symbols.csv').open('w',newline='') as report:
        w=csv.writer(report);w.writerow(('library','sha256','undefined_count','forbidden_count'))
        for name in ['kinetica-physics','kinetica-scenes']:
            archive,=build.rglob(f'lib{name}.a')
            nm_log=f'{name}-undefined.txt'
            run([nm,'-u',archive],nm_log,env=env)
            symbols=[s.split()[-1] for s in (out/nm_log).read_text().splitlines() if re.search(r'\bU\s+[^\s]+$',s)]
            if not symbols: raise RuntimeError(f'No undefined symbol parsed for {name}')
            forbidden=[s for s in symbols if banned.fullmatch(re.sub(r'^__imp_','',s).lstrip('_'))]
            w.writerow((archive.name,digest(archive),len(symbols),len(forbidden)));report.flush()
            if forbidden: raise RuntimeError(f'Forbidden libm symbols: {forbidden}')

    # Missing reference study targets are built externally from the cap-manifold source and linked
    # with its previously measured, unmodified archives and its target-specific CMake compiler flags.
    refdir.mkdir(exist_ok=True)
    listings=[]
    cap_lib_sha={ar.name:digest(ar) for ar in (cap_core,cap_scenes)}
    fallback={'study-groove-friction':'GrooveFriction.cpp','study-state-hash':'StateHash.cpp'}
    for target in group:
        dst=refdir/(target+'.exe'); existing=base_build/'validation/studies'/(target+'.exe')
        if existing.is_file():
            shutil.copy2(existing,dst); origin='cap-manifold-build executable'
        else:
            cpp=fallback[target]
            cwd=base_build/'validation/studies'
            opts=cwd/'CMakeFiles'/(target+'.dir')/'flags.make'
            flags={}
            for line in opts.read_text().splitlines():
                if ' = ' in line:
                    key,val=line.split(' = ',1);flags[key]=shlex.split(val)
            obj=refdir/(target+'.obj')
            run([c['CMAKE_CXX_COMPILER'],*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],
                '-c',base/'validation/studies'/cpp,'-o',obj],target+'-reference-compile.log',env=env,cwd=cwd)
            libraries=([cap_scenes] if target=='study-state-hash' else [])+[cap_core]
            run([c['CMAKE_CXX_COMPILER'],*shlex.split(c['CMAKE_EXE_LINKER_FLAGS']),*shlex.split(c['CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO']),
                 obj,*libraries,'-lpthread','-o',dst],target+'-reference-link.log',env=env,cwd=cwd)
            origin='cap-manifold source and existing cap-manifold archive' 
        listings.append(('cap-manifold',target,origin,digest(dst)))
        candidate=build/'validation/studies'/(target+'.exe')
        listings.append(('combined',target,'combined-core-build executable',digest(candidate)))
    listings.append(('combined','compare-ours','combined-core-build executable',digest(build/'validation/engines/compare-ours.exe')))
    with (out/'exe-hashes.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(('build','target','origin','sha256'));w.writerows(listings)
    if {ar.name:digest(ar) for ar in (cap_core,cap_scenes)}!=cap_lib_sha:raise RuntimeError('reference changed')
    (out/'reference-archives.json').write_text(json.dumps(cap_lib_sha,indent=2))
    for label,d in [('cap-manifold',base_build),('combined',build)]:
        flags=next((d/'src').rglob('kinetica-physics.dir/flags.make'))
        shutil.copyfile(flags,out/f'flags-{label}.txt')
    run([sys.executable,out/'guide_verify.py',refdir,build/'validation/studies',out/'verification','--parity-only'],'verify.log',env=env)
    print((out/'ctest.log').read_text().splitlines()[-4:])
    print((out/'verification/parity.csv').read_text())
finally:
    with tarfile.open(lab/'combined-core-evidence.tgz','w:gz') as archive:
        for p in out.rglob('*'):
            if p.is_file() and p.name!='worktree.tgz':archive.add(p,p.relative_to(out))
