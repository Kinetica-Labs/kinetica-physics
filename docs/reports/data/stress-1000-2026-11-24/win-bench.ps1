param([string]$Mode)
$ErrorActionPreference='Stop'
$ProgressPreference='SilentlyContinue'
$root=Join-Path $env:USERPROFILE 'kp-2326667-mwrev8ox/sieve-20261122-100223'
$w=Join-Path $root 'pyramid1000-nov24-v2'
$lab=Join-Path $root 'engine-anatomy-nov22'
$prior=Join-Path $root 'bucket-energy-and-equilibrium-nov23'
$old='C:/Users/sshuser/CODE/kinetica-physics'
$bin="$old/tools/llvm-mingw-20261117-ucrt-x86_64/bin"
$env:Path="$bin;"+$env:Path
$env:KP_THREADS='1'
$engines=@('kinetica','box3d','bullet','jolt','rp3d')
$parameters=@{kinetica='realtime 1';box3d='1';rp3d='1';bullet='';jolt=''}
New-Item -Type Directory -Path "$w/results" -Force | Out-Null
Set-Location $w
if ($Mode -eq 'build') {
    tar -xzf "$root/pyramid1000-nov24-v2-src.tgz" -C $w
    if ($LASTEXITCODE) {exit $LASTEXITCODE}
    $compiler="$bin/clang++.exe"
    $common=@('-O3','-DNDEBUG','-std=c++17','-march=haswell','-ffp-contract=off','-g','-gcodeview')
    $manifest=[ordered]@{host=$env:COMPUTERNAME;cpu=(Get-CimInstance Win32_Processor | Select-Object -First 1 -ExpandProperty Name);compiler=((& $compiler --version)[0]);affinity=256;threads=1;frames=120;dt='1/60';calls_per_frame=8;sleep=$false;friction=0.5;restitution=0;rows=@{};sources=@{}}
    function Build-One($key,$source,$extra,$archives) {
        $options=@($common)+@($extra)+@("$w/validation/engines/$source")+@($archives)+@("-Wl,--pdb=$w/$key.pdb",'-o',"$w/$key.exe")
        & $compiler @options
        if ($LASTEXITCODE) {throw "build failed for $key"}
        $libraries=@{}; foreach($lib in $archives) {$libraries[(Split-Path -Leaf $lib)]=(Get-FileHash $lib).Hash}
        $manifest.rows[$key]=[ordered]@{sha256=(Get-FileHash "$w/$key.exe").Hash;argv=$parameters[$key];flags=$options;libraries=$libraries}
        "Built $key"
    }
    Build-One 'kinetica' 'ours.cpp' @('-flto',"-I$prior/source/include") @("$prior/build/src/libkinetica-physics.a")
    Build-One 'box3d' 'box3d.cpp' @("-I$old/box3d/include") @("$old/build-box3d/src/libbox3d.a")
    Build-One 'rp3d' 'rp3d.cpp' @('-DIS_RP3D_DOUBLE_PRECISION_ENABLED',"-I$old/rp3d/include") @("$old/build-rp3d-double/libreactphysics3d.a")
    $bullet="$lab/bullet-double-lab/lib"
    Build-One 'bullet' 'stress/bullet.cpp' @('-DBT_USE_DOUBLE_PRECISION',"-I$root/build/anatomy-thirdparty/bullet/src") @("$bullet/libBulletDynamics.a","$bullet/libBulletCollision.a","$bullet/libLinearMath.a")
    $jflags=@('-fno-rtti','-fno-exceptions','-ffp-contract=on','-mavx2','-mbmi','-mpopcnt','-mlzcnt','-mf16c','-mfma','-mfpmath=sse','-DJPH_OBJECT_LAYER_BITS=16','-DJPH_OBJECT_STREAM','-DJPH_USE_AVX','-DJPH_USE_AVX2','-DJPH_USE_CPU_COMPUTE','-DJPH_USE_DX12','-DJPH_USE_F16C','-DJPH_USE_FMADD','-DJPH_USE_LZCNT','-DJPH_USE_SSE4_1','-DJPH_USE_SSE4_2','-DJPH_USE_TZCNT',"-I$root/build/anatomy-thirdparty/jolt")
    Build-One 'jolt' 'stress/jolt.cpp' $jflags @("$lab/jolt-build/libJolt.a")
    foreach($file in Get-ChildItem "$w/validation/engines" -Recurse -File){$manifest.sources[$file.FullName.Substring($w.Length+1)]=(Get-FileHash -LiteralPath $file.FullName).Hash}
    foreach($name in @('SoftStep.cpp','EquilibriumStep.cpp','IslandNormals.cpp')){$manifest.sources["core/$name"]=(Get-FileHash "$prior/source/src/dynamics/time_stepping/$name").Hash}
    $manifest | ConvertTo-Json -Depth 8 | Set-Content -Encoding utf8 "$w/results/manifest.json"
    exit 0
}
$me=[System.Diagnostics.Process]::GetCurrentProcess()
$inheritedAffinity=$me.ProcessorAffinity
$me.ProcessorAffinity=[intptr]256
if ($me.ProcessorAffinity.ToInt64() -ne 256) {throw 'Could not pin the process'}
if($Mode -eq 'stages'){
    Copy-Item "$root/stress-stage-clock.h" "$w/validation/engines/StageClock.h" -Force
    $src=Get-Content "$w/validation/engines/ours.cpp" -Raw -Encoding utf8
    $src=($src.Replace('#include <physics.h>','#include <physics.h>'+[Environment]::NewLine+'#include "StageClock.h"')).Replace('World world;','World world; StageClock stages; world.observer = &stages;').Replace('world.step(Real(dt));','stages.start(); world.step(Real(dt)); stages.store(12);')
    $src | Set-Content -Encoding utf8 "$w/validation/engines/ours-stage.cpp"
    & "$bin/clang++.exe" -O3 -DNDEBUG -std=c++17 -march=haswell -ffp-contract=off -flto "-I$prior/source/include" "$w/validation/engines/ours-stage.cpp" "$prior/build/src/libkinetica-physics.a" -o "$w/stages.exe"
    if($LASTEXITCODE){throw 'stage build failed'}
    $env:STRESS_SCENE='pyramid1000'; $env:STRESS_PASS='time'
    cmd /d /c "stages.exe realtime 1 > results/stage-frames.csv"
    if($LASTEXITCODE){throw 'stage run failed'}
    Get-Content "$w/results/stage-cost.csv"
    Get-Content "$w/results/stage-frames.csv" -Tail 1
    exit 0
}
if($Mode -eq 'accuracy'){
    $src=Get-Content "$w/validation/engines/ours.cpp" -Raw -Encoding utf8
    $depth="$w/validation/engines/depth"
    New-Item -Force -ItemType Directory $depth | Out-Null
    Copy-Item "$root/stress-depth-readings.h" "$depth/StressReadings.h" -Force
    Copy-Item "$w/validation/engines/stress/Stress1000.h" "$depth/Stress1000.h" -Force
    $layout="$w/validation/engines/stress/CubeLayout.h"
    if(Test-Path $layout){Copy-Item $layout "$depth/CubeLayout.h" -Force}
    $src=$src.Replace('"stress/Stress1000.h"','"depth/Stress1000.h"')
    $setting='if (const char* mode=std::getenv("KP_DEPTH_UM")) world.soft.recheckDepth=std::atoi(mode)*Real(1e-6);'
    $src=$src.Replace('world.soft.contactUpdates = 1;', 'world.soft.contactUpdates = 1; '+$setting)
    $src | Set-Content -Encoding utf8 "$w/validation/engines/ours-depth.cpp"
    & "$bin/clang++.exe" -O3 -DNDEBUG -std=c++17 -march=haswell -ffp-contract=off -flto "-I$prior/source/include" "$w/validation/engines/ours-depth.cpp" "$prior/build/src/libkinetica-physics.a" -o "$w/accuracy.exe"
    if($LASTEXITCODE){throw 'accuracy build failed'}
    foreach($label in @('default','500','250')){
        if($label -eq 'default'){Remove-Item Env:KP_DEPTH_UM -ErrorAction SilentlyContinue}else{$env:KP_DEPTH_UM=$label}
        foreach($scene in @('pyramid1000','rain1000')){
            $env:STRESS_SCENE=$scene
            foreach($pass in @('verify','time')){
                $env:STRESS_PASS=$pass
                $out="results/depth-$label-$scene-$pass.csv"
                cmd /d /c "accuracy.exe realtime 1 > $out 2>results/depth-$label-$scene-$pass-violations.txt"
                if($LASTEXITCODE){throw "failed $label $scene $pass"}
                $rows=@(Import-Csv $out)
                if($pass -eq 'verify'){
                    if($rows.Count -ne 121){throw 'incomplete positions'}
                    if($label -eq 'default' -and (Get-FileHash $out).Hash -ne (Get-FileHash "results/$scene-kinetica-verify-0.csv").Hash){throw 'incorrect default replay'}
                    $hash=$rows[-1].state_hash
                } elseif($rows.Count -ne 120 -or $rows[-1].final_hash -ne $hash){throw 'time and geometry disagree'}
                "$scene $label $pass $hash"
            }
        }
    }
    exit 0
}
if($Mode -eq 'candidates' -or $Mode -eq 'chunks'){
    $flags=@('-O3','-DNDEBUG','-std=c++17','-march=haswell','-ffp-contract=off','-flto',"-I$prior/source/include")
    $chunkTrial=$Mode -eq 'chunks'
    $variants=if($chunkTrial){@('unbatched','chunks','leaves','stream')}else{@('base','signs','axes')}
    if($chunkTrial){
        New-Item -Force -ItemType Directory "$w/chunks/include/collision" | Out-Null
        Copy-Item "$root/stress-candidate-PairFinder.h" "$w/chunks/include/collision/PairFinder.h" -Force
        New-Item -Force -ItemType Directory "$w/stream/include/collision" | Out-Null
        Copy-Item "$root/stress-stream-PairFinder.h" "$w/stream/include/collision/PairFinder.h" -Force
    }
    foreach($key in $variants){
        $f=if($key -eq 'base'){"$prior/source/src/collision/narrowphase/Sat.cpp"}else{"$root/stress-candidate-$key.cpp"}
        $extra=@()
        if($key -eq 'unbatched'){ $f="$prior/source/src/collision/PairFinder.cpp" }
        if($key -eq 'chunks'){ $f="$root/stress-candidate-PairFinder.cpp"; $extra=@("-I$w/chunks/include") }
        if($key -eq 'stream'){ $f="$root/stress-stream-PairFinder.cpp"; $extra=@("-I$w/stream/include") }
        & "$bin/clang++.exe" @extra @flags -c $f -o "$w/opt-$key.obj"
        if($LASTEXITCODE){throw "could not build $key"}
        & "$bin/clang++.exe" @flags "$w/validation/engines/ours.cpp" "$w/opt-$key.obj" "$prior/build/src/libkinetica-physics.a" -o "$w/opt-$key.exe"
        if($LASTEXITCODE){throw "could not link $key"}
        "$key exe "+(Get-FileHash "$w/opt-$key.exe").Hash+" src "+(Get-FileHash $f).Hash | Add-Content -Encoding ascii "$w/results/opt-build.txt"
        $env:STRESS_PASS='verify'
        foreach($scene in @('pyramid1000','rain1000')){
            $env:STRESS_SCENE=$scene
            cmd /d /c "opt-$key.exe realtime 1 > results/opt-$key-$scene-verify.csv"
            if($LASTEXITCODE){throw "verification failed $key on $scene"}
            $gold=Get-FileHash "results/$scene-kinetica-verify-0.csv"
            $got=Get-FileHash "results/opt-$key-$scene-verify.csv"
            if($got.Hash -ne $gold.Hash){throw "different frame sequence for $key on $scene"}
            "Matched 121 frames: $scene $key"
        }
    }
    $orders=@('base,signs,axes','axes,base,signs','signs,axes,base','axes,signs,base','base,axes,signs','signs,base,axes')
    if($chunkTrial){$orders=@('unbatched,chunks,stream,leaves','chunks,leaves,unbatched,stream','leaves,stream,chunks,unbatched','stream,unbatched,leaves,chunks')}
    $env:STRESS_PASS='time'
    $lines=@('block,slot,scene,variant,mean_ms,final_hash')
    $table=if($chunkTrial){'opt-chunks-pairs.csv'}else{'opt-pairs.csv'}
    for($block=0;$block -lt $orders.Count;$block++){
        $scenes=if($block%2 -eq 0){@('pyramid1000','rain1000')}else{@('rain1000','pyramid1000')}
        foreach($scene in $scenes){
            $env:STRESS_SCENE=$scene; $slot=0
            $reference=@(Import-Csv "results/$scene-kinetica-verify-0.csv")[-1].state_hash
            foreach($key in $orders[$block].Split(',')){
                $slot++
                $to="results/opt-$key-$scene-time-$($block+1).csv"
                cmd /d /c "opt-$key.exe realtime 1 > $to"
                if($LASTEXITCODE){throw "timing failed $key"}
                $rows=@(Import-Csv $to)
                if($rows.Count -ne 120 -or $rows[-1].final_hash -ne $reference){throw "timing hash failed $key"}
                $us=0.0; foreach($r in $rows){$us += [double]::Parse($r.step_us,[Globalization.CultureInfo]::InvariantCulture)}
                $ms=($us/120000).ToString('F6',[Globalization.CultureInfo]::InvariantCulture)
                $line="$($block+1),$slot,$scene,$key,$ms,$reference"
                $lines+=$line; Write-Output $line
            }
        }
        $lines | Set-Content -Encoding ascii "$w/results/$table"
    }
    exit 0
}
if ($Mode -eq 'attach') {
    $v='C:/Program Files (x86)/Intel/oneAPI/vtune/latest/bin64/vtune.exe'
    $env:STRESS_SCENE='pyramid1000'
    $env:STRESS_PASS='time'
    Remove-Item Env:STRESS_POSES -ErrorAction SilentlyContinue
    $key='kinetica'
    $collection="$w/attached-vtune-$key"
    $checked="$w/results/attached-$key-frames.csv"
    $process=Start-Process "$w/$key.exe" -ArgumentList @('realtime','1') -NoNewWindow -PassThru -RedirectStandardOutput $checked -RedirectStandardError "$w/results/attached-$key.err"
    $me.ProcessorAffinity=$inheritedAffinity
    # Запуск ядра и создание пула закончены до присоединения агента VTune.
    Start-Sleep -Milliseconds 1300
    & $v -collect hotspots -knob sampling-mode=sw -duration 5 -result-dir $collection -target-pid $process.Id > "$w/results/attached-collection.txt"
    if($LASTEXITCODE){throw "VTune attach failed $LASTEXITCODE"}
    if(-not $process.WaitForExit(75000)){throw 'Profiled step did not complete within 75 seconds after detach'}
    $samples=@(Import-Csv $checked)
    $reference=@(Import-Csv "$w/results/pyramid1000-$key-verify-0.csv")[-1].state_hash
    if($samples.Count -ne 120 -or $samples[-1].final_hash -ne $reference){throw 'Profiled workload mismatch'}
    & $v -report hotspots -r $collection -group-by function -format csv -csv-delimiter comma -report-output "$w/results/attached-hotspots.csv"
    if($LASTEXITCODE){throw "VTune hotspot report failed $LASTEXITCODE"}
    & $v -report summary -r $collection -report-output "$w/results/attached-summary.txt"
    exit $LASTEXITCODE
}
if ($Mode -eq 'profile') {
    $v='C:/Program Files (x86)/Intel/oneAPI/vtune/latest/bin64/vtune.exe'
    $me.ProcessorAffinity=$inheritedAffinity
    foreach($key in @('kinetica','box3d')){
        $exe="$w/$key.exe"
        $profile="$w/checked-vtune-$key"
        $batch="$w/checked-$key.cmd"
        $checked="$w/results/profile-$key-frames.csv"
        @('@echo off','set KP_THREADS=1','set STRESS_SCENE=pyramid1000','set STRESS_PASS=time','set STRESS_POSES=',
            "start `"`" /b /wait /affinity 100 `"$exe`" $($parameters[$key]) > `"$checked`"",
            'exit /b %ERRORLEVEL%') | Set-Content -Encoding ascii $batch
        if(-not(Test-Path "$profile/config")){
            & $v -collect hotspots -knob sampling-mode=sw -result-dir $profile -- $env:ComSpec /d /c $batch > "$w/results/profile-$key-out.txt"
            if($LASTEXITCODE){throw "VTune collection failed $key $LASTEXITCODE"}
        }
        $samples=@(Import-Csv $checked)
        $reference=@(Import-Csv "$w/results/pyramid1000-$key-verify-0.csv")[-1].state_hash
        if($samples.Count -ne 120 -or $samples[-1].final_hash -ne $reference){throw "Wrong VTune workload for $key"}
        & $v -report hotspots -r $profile -group-by function -format csv -csv-delimiter comma -report-output "$w/results/vtune-$key.csv"
        if($LASTEXITCODE){throw "VTune report failed $key $LASTEXITCODE"}
        & $v -report summary -r $profile -report-output "$w/results/vtune-$key-summary.txt"
        if($LASTEXITCODE){throw "VTune summary failed $key $LASTEXITCODE"}
    }
    exit 0
}
if ($Mode -eq 'metadata') {
    $prefix=(Resolve-Path "$prior/source").Path
    $sourceHashes=foreach($dir in @('include','src')) {
        foreach($f in Get-ChildItem "$prefix/$dir" -Recurse -File -Include *.cpp,*.h,*.inl){
            [pscustomobject]@{relative=$f.FullName.Substring($prefix.Length+1).Replace('\','/');sha256=(Get-FileHash -LiteralPath $f.FullName).Hash}
        }
    }
    $sourceHashes | Export-Csv -NoTypeInformation -Encoding utf8 "$w/results/kinetica-sources.csv"
    foreach($pair in @(@('box3d',"$old/box3d"),@('rp3d',"$old/rp3d"),@('jolt',"$root/build/anatomy-thirdparty/jolt"),@('bullet',"$root/build/anatomy-thirdparty/bullet"))){
        if(Test-Path "$($pair[1])/.git"){git -C $pair[1] rev-parse HEAD | Set-Content -Encoding ascii "$w/results/$($pair[0])-revision.txt"}
    }
    exit 0
}
if ($Mode -eq 'rim') {
    Copy-Item "$root/pyramid1000-before-overlay.h" "$w/PreviousOverlay.h" -Force
    Copy-Item "$root/pyramid1000-rim-timing.cpp" "$w/tilted_timing.cpp" -Force
    & "$bin/clang++.exe" -O3 -DNDEBUG -std=c++17 -march=haswell -ffp-contract=off -flto "-I$w" "-I$prior/source" "-I$prior/source/include" tilted_timing.cpp "$prior/build/examples/libkinetica-scenes.a" "$prior/build/src/libkinetica-physics.a" -o rim-timing.exe
    if ($LASTEXITCODE) {exit $LASTEXITCODE}
    cmd /d /c "rim-timing.exe >results/rim-timing.partial 2>results/rim-timing.err"
    if ($LASTEXITCODE) {exit $LASTEXITCODE}
    Move-Item -Force results/rim-timing.partial results/rim-timing.csv
    exit 0
}
function Measure-One([string]$pass,[string]$scene,[string]$engine,[int]$block) {
    $dest="results/$scene-$engine-$pass-$block"
    if (Test-Path "$dest.csv") {return}
    $env:STRESS_SCENE=$scene
    $env:STRESS_PASS=$pass
    if ($pass -eq 'verify') {$env:STRESS_POSES="$w/$dest-poses.csv"} else {Remove-Item Env:STRESS_POSES -ErrorAction SilentlyContinue}
    $cmd="$engine.exe "+$parameters[$engine]+" >$dest.partial 2>$dest.err"
    cmd /d /c $cmd
    if ($LASTEXITCODE) {$code=$LASTEXITCODE;Get-Content "$dest.err" -Tail 5;throw "$cmd : native exit $code"}
    $lines=[IO.File]::ReadAllLines("$w/$dest.partial")
    $expected=if($pass -eq 'time'){121}else{122}
    if($lines.Count -ne $expected){throw "$dest $($lines.Count) rows"}
    Move-Item "$dest.partial" "$dest.csv"
    "$scene $engine $pass $block $($lines[-1])"
}
if ($Mode -eq 'pairs') {
    $random=[System.Random]::new(20261124)
    $lines=@('block,slot,scene,engine')
    for($b=1;$b -le 12;$b++){
        $scenes=if($random.Next(2) -eq 0){@('pyramid1000','rain1000')}else{@('rain1000','pyramid1000')}
        foreach($scene in $scenes){
            $order=if($random.Next(2) -eq 0){@('box3d','jolt')}else{@('jolt','box3d')}
            for($slot=0;$slot -lt 2;$slot++){$lines+="$b,$($slot+1),$scene,$($order[$slot])"}
        }
    }
    $lines | Set-Content -Encoding ascii "$w/results/confirm-order.csv"
    foreach($job in Import-Csv "$w/results/confirm-order.csv"){
        Measure-One 'time' $job.scene $job.engine (100+[int]$job.block)
    }
    exit 0
}
if ($Mode -eq 'verify') {
    foreach($scene in @('pyramid1000','rain1000')){
        foreach($key in $engines){Measure-One 'verify' $scene $key 0}
    }
    exit 0
}
if ($Mode -eq 'time') {
    @('block,slot,scene,engine') | Set-Content -Encoding ascii "$w/results/order.csv"
    # Пять циклических порядков: каждый движок один раз занимает каждое место.
    $order=@('rp3d','kinetica','jolt','box3d','bullet')
    for($block=0;$block -lt 5;$block++){
        $scenes=if($block%2 -eq 0){@('pyramid1000','rain1000')}else{@('rain1000','pyramid1000')}
        foreach($scene in $scenes){
            for($slot=0;$slot -lt 5;$slot++){
                $key=$order[($slot+$block)%5]
                "$($block+1),$($slot+1),$scene,$key" | Add-Content -Encoding ascii "$w/results/order.csv"
                Measure-One 'time' $scene $key ($block+1)
            }
        }
    }
    exit 0
}
throw "Unknown mode $Mode"
