# Run from the Windows study after the 0/3 geometry passes. Each episode is 120 frames at 60 Hz.
param([Parameter(Mandatory=$true)][string]$Experiment, [string]$RestSource)
$ErrorActionPreference = 'Stop'
$study = Join-Path $Experiment 'rest55-repair'
$prior = Join-Path $Experiment 'results'
$out = Join-Path $Experiment 'support-time-10-v1'
if ($RestSource) {$out += '-rest'}
New-Item -Force -ItemType Directory $out | Out-Null
$lock = [IO.File]::Open("$out/series.lock",'OpenOrCreate','ReadWrite','None')
try {
    $env:KP_THREADS = '1'
    $env:STRESS_PASS = 'time'
    Remove-Item Env:KP_STUDY_BALANCE -ErrorAction SilentlyContinue
    $proc = [Diagnostics.Process]::GetCurrentProcess()
    $proc.ProcessorAffinity = [intptr]256
    if ($proc.ProcessorAffinity.ToInt64() -ne 256) {throw 'CPU affinity'}
    Set-Location $study
    $hashes = @{}
    if ($RestSource) {
        $scenes = @('rest55')
        $steps = 1
        $command = 'rest-time.exe --rest-time'
        $priorBuild = Join-Path (Split-Path $Experiment) 'bucket-energy-and-equilibrium-nov23'
        Set-Location $out
        & clang++.exe -O3 -DNDEBUG -std=c++17 -march=haswell -ffp-contract=off -flto "-I$priorBuild/source/include" "-I$study/gui/viewer" $RestSource "$study/gui/viewer/Viewer.cpp" "$study/gui/viewer/ImpulseTool.cpp" "$study/SoftStep.obj" "$priorBuild/build/src/libkinetica-physics.a" -o rest-time.exe
        if ($LASTEXITCODE) {throw 'rest timing compile'}
        $binary = (Get-FileHash ./rest-time.exe).Hash
        foreach ($mode in @(0,3)) {
            $env:KP_STUDY_SUPPORT = [string]$mode
            cmd.exe /d /c "$command > ref-$mode.csv"
            if ($LASTEXITCODE) {throw 'rest reference process'}
            $rows = @(Import-Csv "./ref-$mode.csv")
            if ($rows.Count -ne 120) {throw 'rest reference incomplete'}
            $hashes["rest55-$mode"] = $rows[-1].final_hash
        }
        if ($hashes['rest55-0'] -ne $hashes['rest55-3']) {throw 'resting states differ'}
    } else {
        $scenes = @('pyramid1000','rain1000')
        $steps = 8
        $command = 'stress.exe realtime 1'
        $binary = (Get-FileHash ./stress.exe).Hash
        $build = Import-Csv "$prior/rest55-fix-build.csv" | Where-Object { [IO.Path]::GetFileName($_.Path) -eq 'stress.exe' }
        if ($binary -ne $build.Hash) {throw 'binary differs from geometry pass'}
        foreach ($scene in $scenes) {
            foreach ($mode in @(0,3)) {
                $file = if ($mode -eq 0) {"support-0-$scene-verify.csv"} else {"rest55-3-$scene.csv"}
                $rows = @(Import-Csv "$prior/$file")
                if ($rows.Count -ne 121 -or [int]$rows[-1].frame -ne 120) {throw "incomplete geometry $file"}
                $hashes["$scene-$mode"] = $rows[-1].state_hash
            }
        }
    }
    $order = @(foreach ($block in 1..10) {
        $sceneOrder = if (($block % 2) -or $RestSource) {$scenes} else {@('rain1000','pyramid1000')}
        foreach ($scene in $sceneOrder) {
            $modes = if (($scene -eq 'pyramid1000') -eq [bool]($block % 2)) {@(0,3)} else {@(3,0)}
            for ($i = 0; $i -lt 2; $i++) {
                [pscustomobject]@{block=$block;scene=$scene;slot=$i+1;support=$modes[$i]}
            }
        }
    })
    $order | Export-Csv "$out/order.csv" -NoTypeInformation
    [ordered]@{binary_sha256=$binary;affinity=$proc.ProcessorAffinity.ToInt64();threads=1;
        cpu=(Get-CimInstance Win32_Processor | Select-Object -First 1 -ExpandProperty Name);
        compiler=((& clang++.exe --version)[0]);power=(powercfg /getactivescheme | Out-String).Trim();
        measured_seconds=2;frames=120;world_steps_per_frame=$steps;expected_hashes=$hashes} |
        ConvertTo-Json | Set-Content -Encoding UTF8 "$out/manifest.json"

    function Check-Frames($path, $expected) {
        $rows = @(Import-Csv $path)
        if ($rows.Count -ne 120) {throw "expected 120 rows in $path"}
        for ($i = 0; $i -lt $rows.Count; $i++) {
            $row = $rows[$i]
            $us = [double]::Parse($row.step_us,[Globalization.CultureInfo]::InvariantCulture)
            if ([int]$row.frame -ne ($i+1) -or $row.final_hash -ne $expected -or $us -le 0 -or
                [double]::IsNaN($us) -or [double]::IsInfinity($us)) {throw "invalid row $i in $path"}
        }
    }
    function Record-Run($name, $scene, $mode) {
        $path = "$out/$name.csv"
        $expected = $hashes["$scene-$mode"]
        if (Test-Path $path) {Check-Frames $path $expected; return}
        $env:STRESS_SCENE = $scene
        $env:KP_STUDY_SUPPORT = [string]$mode
        cmd.exe /d /c "$command > `"$path.pending`" 2> `"$out/$name.log`""
        if ($LASTEXITCODE -ne 0) {throw "$name exited $LASTEXITCODE"}
        Check-Frames "$path.pending" $expected
        Move-Item "$path.pending" $path
    }
    if (-not $RestSource) {foreach ($mode in @(0,3)) {Record-Run "warmup-$mode" 'pyramid1000' $mode}}
    foreach ($item in $order) {
        $name = "$($item.scene)-block-$($item.block)-mode-$($item.support)"
        Record-Run $name $item.scene $item.support
        Write-Output $name
    }
} finally {
    $lock.Dispose()
}
