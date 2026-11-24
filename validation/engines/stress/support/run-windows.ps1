param([string]$Phase='pyramid')
$ErrorActionPreference='Stop'
$root=Join-Path $env:USERPROFILE 'kp-2326667-mwrev8ox/sieve-20261122-100223'
$prior=Join-Path $root 'bucket-energy-and-equilibrium-nov23'
$w=Join-Path $root 'pyramid1000-nov24-v2'
$s=Join-Path $w 'support-study'
if($Phase -in @('repair','corners','drag')){$s=Join-Path $w 'rest55-repair'}
$out=Join-Path $w 'results'
$bin='C:/Users/sshuser/CODE/kinetica-physics/tools/llvm-mingw-20261117-ucrt-x86_64/bin'
$env:Path="$bin;"+$env:Path
$env:KP_THREADS='1'
$me=[System.Diagnostics.Process]::GetCurrentProcess()
$me.ProcessorAffinity=[intptr]256
if($me.ProcessorAffinity.ToInt64() -ne 256){throw 'CPU affinity'}
New-Item -Force -ItemType Directory $s | Out-Null
Set-Location $s
tar -xzf "$root/stress-support-study.tgz"
if($LASTEXITCODE){throw 'sources'}
$flags=@('-O3','-DNDEBUG','-std=c++17','-march=haswell','-ffp-contract=off','-flto',"-I$prior/source/include","-I$prior/source/src/dynamics/time_stepping","-I$s")
$lib="$prior/build/src/libkinetica-physics.a"
if($Phase -eq 'pyramid'){
    $code=Get-Content "$prior/source/src/dynamics/time_stepping/SoftStep.cpp" -Raw -Encoding utf8
    function EditOnce([string]$source,[string]$old,[string]$replacement) {
        if($source.IndexOf($old) -lt 0 -or $source.IndexOf($old) -ne $source.LastIndexOf($old)){throw "Missing or repeated insertion $old"}
        return $source.Replace($old,$replacement)
    }
    $code=EditOnce $code '#include "EquilibriumStep.h"' ('#include "EquilibriumStep.h"'+[Environment]::NewLine+'#include "SupportAudit.h"')
    $code=EditOnce $code 'bool bounces;' 'Vector3 gravity; bool bounces;'
    $code=EditOnce $code 'bounceSpeed, edgeSlop, observer,' 'bounceSpeed, edgeSlop, observer, gravity,'
    $code=EditOnce $code 'missed.forget(), moving.advance(h);' ('support_study::studyPass(contacts, moving.bodies, moving.states, gravity, h);'+[Environment]::NewLine+'            missed.forget(), moving.advance(h);')
    $code | Set-Content -Encoding utf8 "$s/SoftStep.cpp"
    & "$bin/clang++.exe" @flags -c "$s/SoftStep.cpp" -o "$s/SoftStep.obj"
    if($LASTEXITCODE){throw 'support step compile'}
    & "$bin/clang++.exe" @flags "$w/validation/engines/ours-depth.cpp" "$s/SoftStep.obj" $lib -o "$s/support.exe"
    if($LASTEXITCODE){throw 'support link'}
    Get-FileHash .\SupportSweep.h,.\SupportAudit.h,.\SoftStep.cpp,.\support.exe | Select-Object Path,Hash | Export-Csv -NoTypeInformation "$out/support-build.csv"
    foreach($variant in @('0','1','2')){
        $env:KP_STUDY_SUPPORT=$variant
        foreach($scene in @('pyramid1000','rain1000')){
            $env:STRESS_SCENE=$scene
            foreach($pass in @('verify','time')){
                if($pass -eq 'verify'){$env:KP_STUDY_BALANCE='1'}else{Remove-Item Env:KP_STUDY_BALANCE -ErrorAction SilentlyContinue}
                $env:STRESS_PASS=$pass
                $name="$out/support-$variant-$scene-$pass"
                cmd /d /c "support.exe realtime 1 > `"$name.csv`" 2> `"$name.log`""
                if($LASTEXITCODE){throw "$variant $scene $pass"}
                $rows=@(Import-Csv "$name.csv")
                if($pass -eq 'verify'){
                    if($rows.Count -ne 121){throw "positions $variant $scene"}
                    $hash=$rows[-1].state_hash
                    if($variant -eq '0' -and (Get-FileHash "$name.csv").Hash -ne (Get-FileHash "$out/$scene-kinetica-verify-0.csv").Hash){throw 'wrong control trajectory'}
                } elseif($rows.Count -ne 120 -or $rows[-1].final_hash -ne $hash){throw 'wrong timing trajectory'}
                Write-Output "$variant $scene $pass $hash"
            }
        }
    }
} elseif($Phase -eq 'repair'){
    Copy-Item "$w/support-study/SoftStep.cpp" "$s/SoftStep.cpp" -Force
    & "$bin/clang++.exe" @flags -c "$s/SoftStep.cpp" -o "$s/SoftStep.obj"
    if($LASTEXITCODE){throw 'revised support compile'}
    foreach($job in @(@('rest55',"$s/Rest55.cpp"),@('stress',"$w/validation/engines/ours-depth.cpp"),@('balance',"$s/Balance.cpp"))){
        & "$bin/clang++.exe" @flags $job[1] "$s/SoftStep.obj" $lib -o "$s/$($job[0]).exe"
        if($LASTEXITCODE){throw "compile $($job[0])"}
    }
    Get-FileHash .\SupportSweep.h,.\SupportAudit.h,.\SoftStep.cpp,.\rest55.exe,.\stress.exe | Select-Object Path,Hash | Export-Csv -NoTypeInformation "$out/rest55-fix-build.csv"
    foreach($variant in @('3','4')){
        $env:KP_STUDY_SUPPORT=$variant
        Remove-Item Env:KP_STUDY_BALANCE -ErrorAction SilentlyContinue
        cmd /d /c "rest55.exe 2100 > `"$out/rest55-$variant.csv`""
        if($LASTEXITCODE){throw "rest $variant"}
        Get-Content "$out/rest55-$variant.csv" -Tail 1
        cmd /d /c "rest55.exe 1500 push > `"$out/rest55-$variant-push.csv`""
        if($LASTEXITCODE){throw "pulse $variant"}
        $env:KP_STUDY_BALANCE='1'
        cmd /d /c "balance.exe > `"$out/rest55-$variant-balance.csv`" 2> `"$out/rest55-$variant-balance.log`""
        if($LASTEXITCODE){throw "balance $variant"}
        foreach($scene in @('pyramid1000','rain1000')){
            $env:STRESS_SCENE=$scene; $env:STRESS_PASS='verify'
            cmd /d /c "stress.exe realtime 1 > `"$out/rest55-$variant-$scene.csv`" 2> `"$out/rest55-$variant-$scene.log`""
            if($LASTEXITCODE){throw "stress $variant $scene"}
            Get-Content "$out/rest55-$variant-$scene.csv" -Tail 1
        }
    }
} elseif($Phase -eq 'drag'){
    & "$bin/clang++.exe" @flags "-I$s/gui/viewer" "$s/Drag.cpp" "$s/gui/viewer/Viewer.cpp" "$s/gui/viewer/ImpulseTool.cpp" "$s/SoftStep.obj" $lib -o "$s/drag.exe"
    if($LASTEXITCODE){throw 'mouse path link'}
    foreach($variant in @('3','2','0')){
        $env:KP_STUDY_SUPPORT=$variant
        cmd /d /c "drag.exe > `"$out/rest55-mouse-$variant.csv`""
        if($LASTEXITCODE){throw "mouse $variant"}
        Get-Content "$out/rest55-mouse-$variant.csv"
    }
} elseif($Phase -eq 'corners'){
    & "$bin/clang++.exe" @flags "$s/Rest55.cpp" "$s/SoftStep.obj" $lib -o "$s/corners.exe"
    if($LASTEXITCODE){throw 'corner measurement build'}
    foreach($variant in @('0','2','3')){
        $env:KP_STUDY_SUPPORT=$variant
        cmd /d /c "corners.exe 2100 > `"$out/rest55-$variant-corners.csv`""
        if($LASTEXITCODE){throw "corners $variant"}
    }
    cmd /d /c "corners.exe 1500 push > `"$out/rest55-3-pulse-corners.csv`""
    if($LASTEXITCODE){throw 'corner pulse'}
    Get-Content "$out/rest55-3-pulse-corners.csv" -Tail 1
} elseif($Phase -eq 'rest'){
    & "$bin/clang++.exe" @flags "$s/Rest55.cpp" "$s/SoftStep.obj" $lib -o "$s/rest55.exe"
    if($LASTEXITCODE){throw 'rest55 link'}
    foreach($variant in @('0','2')){
        $env:KP_STUDY_SUPPORT=$variant
        cmd /d /c "rest55.exe 2100 > `"$out/rest55-$variant.csv`""
        if($LASTEXITCODE){throw "rest55 $variant"}
        Get-Content "$out/rest55-$variant.csv" -Tail 1
    }
} elseif($Phase -eq 'balance'){
    & "$bin/clang++.exe" @flags "$s/Balance.cpp" "$s/SoftStep.obj" $lib -o "$s/balance.exe"
    if($LASTEXITCODE){throw 'balance link'}
    foreach($variant in @('0','1','2')){
        $env:KP_STUDY_SUPPORT=$variant
        $env:KP_STUDY_BALANCE='1'
        cmd /d /c "balance.exe > `"$out/support-$variant-balance.csv`" 2> `"$out/support-$variant-balance.log`""
        if($LASTEXITCODE){throw "balance process $variant"}
        Get-Content "$out/support-$variant-balance.csv"
    }
} else { throw "unknown phase $Phase" }
exit 0
