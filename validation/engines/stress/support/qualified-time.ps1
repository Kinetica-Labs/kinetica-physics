# Сравнение флага World::shockPropagation в одной штатной сборке, после проверки геометрии.
param([Parameter(Mandatory=$true)][string]$Build,
      [Parameter(Mandatory=$true)][string]$Geometry,
      [Parameter(Mandatory=$true)][string]$Output)
$ErrorActionPreference = 'Stop'
New-Item -Force -ItemType Directory $Output | Out-Null
$env:KP_THREADS = '1'
$env:STRESS_PASS = 'time'
$cpuMask = 256
$parent = [Diagnostics.Process]::GetCurrentProcess()
$parent.ProcessorAffinity = [intptr]$cpuMask
$stress = Join-Path $Build 'validation/engines/compare-ours.exe'
$rest = Join-Path $Build 'validation/studies/study-support-switch.exe'

$expected = @{}
foreach ($scene in @('pyramid1000','rain1000')) {
    foreach ($mode in @('off','on')) {
        $ref = @(Import-Csv "$Geometry/$scene-$mode.csv")
        if ($ref.Count -ne 121 -or $ref[-1].frame -ne 120) {throw "incomplete geometry for $scene $mode"}
        $expected["$scene-$mode"] = $ref[-1].state_hash
    }
}
$order = @(foreach ($pair in 1..6) {
    $scenes = if ($pair % 2) {@('pyramid1000','rain1000','rest55')} else {@('rest55','rain1000','pyramid1000')}
    foreach ($scene in $scenes) {
        $modes = if ($pair % 2) {@('off','on')} else {@('on','off')}
        for ($i=0; $i -lt 2; ++$i) {[pscustomobject]@{scene=$scene;pair=$pair;slot=$i+1;mode=$modes[$i]}}
    }
})
$order | Export-Csv "$Output/order.csv" -NoTypeInformation

function Sample ($scene, $mode, $name) {
    $env:STRESS_SCENE=$scene
    $exe=$stress
    $argv=@('realtime','1',"--shock-$mode")
    if ($scene -eq 'rest55') {$exe=$rest; $argv=@('--rest-time',"--shock-$mode")}
    $file="$Output/$name.csv"
    $p=Start-Process -FilePath $exe -ArgumentList $argv -PassThru -NoNewWindow -RedirectStandardOutput "$file.pending" -RedirectStandardError "$file.log"
    $p.Handle | Out-Null
    if ($p.ProcessorAffinity.ToInt64() -ne $cpuMask) {throw 'child affinity'}
    $p.WaitForExit()
    if ($p.ExitCode -ne 0) {throw "failed: $name"}
    $rows=@(Import-Csv "$file.pending")
    if ($rows.Count -ne 120) {throw "incomplete: $name"}
    $key="$scene-$mode"
    if ($scene -eq 'rest55' -and !$expected.ContainsKey($key)) {$expected[$key]=$rows[-1].final_hash}
    for ($i=0; $i -lt 120; ++$i) {
        $us=[double]::Parse($rows[$i].step_us,[Globalization.CultureInfo]::InvariantCulture)
        if ($rows[$i].final_hash -ne $expected[$key] -or [int]$rows[$i].frame -ne $i+1 -or !($us -gt 0) -or [double]::IsInfinity($us)) {throw "invalid frame $i in $name"}
    }
    Move-Item -Force "$file.pending" $file
}
foreach ($mode in @('off','on')) {Sample 'rest55' $mode "warm-rest-$mode"; Sample 'pyramid1000' $mode "warm-collapse-$mode"}
if ($expected['rest55-on'] -ne $expected['rest55-off']) {throw 'rest trajectory differs'}
foreach ($job in $order) {
    Sample $job.scene $job.mode "$($job.scene)-$($job.pair)-$($job.mode)"
    Write-Output "$($job.scene) $($job.pair) $($job.mode)"
}
[ordered]@{threads=1;affinity=$cpuMask;frames=120;blocks=6;Hz=60;cpu=(Get-CimInstance Win32_Processor | Select-Object -First 1 -ExpandProperty Name);
    compiler=((& clang++.exe --version)[0]);hashes=$expected;stress_binary_SHA256=(Get-FileHash $stress).Hash;
    rest_binary_SHA256=(Get-FileHash $rest).Hash} | ConvertTo-Json | Set-Content -Encoding utf8 "$Output/manifest.json"
