# Run from the Windows checkout after preparing the overlay described in the report.
param([string] $Overlay = 'build/arena-study', [string] $Out = 'allocation-lab', [long] $Affinity = 0x100)
$ErrorActionPreference = 'Stop'
$process = Get-Process -Id $PID
$process.ProcessorAffinity = [IntPtr]$Affinity
$env:KP_THREADS = '1'
$env:KP_GRIP_PROFILE = '0'
$env:KP_GRIP_TIMING = '0'
$kernel = 'build/src/libkinetica-physics.a'
$flags = @('-std=c++17','-O3','-DNDEBUG','-g','-gcodeview','-ffp-contract=off',"-I$Overlay/include",'-Iinclude','-Ivalidation/studies/memory')
$variants = @('stock','reuse','heap')
function Native([string] $program, [string[]] $params) {
    & $program @params
    if ($LASTEXITCODE -ne 0) { throw "$program failed ($LASTEXITCODE): $params" }
}
New-Item -Force -ItemType Directory $Out | Out-Null
Native 'clang++' ($flags + @('-DKP_MEMORY_COUNTERS','-c',"$Overlay/validation/studies/GripLimit.cpp",'-o',"$Out/probe.obj"))
Native 'clang++' ($flags + @('-c','validation/studies/memory/MemoryCounters.cpp','-o',"$Out/counters.obj"))
Native 'clang++' ($flags + @('-c','validation/studies/GripLimit.cpp','-o',"$Out/timing.obj"))
foreach ($v in $variants) {
    $dir = "$Out/$v"
    New-Item -Force -ItemType Directory $dir | Out-Null
    $normal = "$dir/normal.a"
    Copy-Item $kernel $normal -Force
    if ($v -eq 'reuse') {
        foreach ($file in @('joints/Joint.cpp','dynamics/time_stepping/SoftStep.cpp')) {
            $obj = "$dir/$(Split-Path -Leaf $file).obj"
            Native 'clang++' ($flags + @('-c',"$Overlay/src/$file",'-o',$obj))
            Native 'llvm-ar' @('r',$normal,$obj)
        }
    }
    $extra = @()
    if ($v -eq 'heap') {
        $extra += '-DKP_FORCE_HEAP'
        Native 'clang++' ($flags + $extra + @('-c',"$Overlay/src/math/StepMemory.cpp",'-o',"$dir/StepMemory.cpp.obj"))
        Native 'llvm-ar' @('r',$normal,"$dir/StepMemory.cpp.obj")
    }
    $audited = "$dir/counted.a"
    Copy-Item $normal $audited -Force
    Native 'clang++' ($flags + $extra + @('-DKP_MEMORY_COUNTERS','-c',"$Overlay/src/math/StepMemory.cpp",'-o',"$dir/StepMemory.cpp.obj"))
    Native 'llvm-ar' @('r',$audited,"$dir/StepMemory.cpp.obj")
    Native 'clang++' @("$Out/probe.obj","$Out/counters.obj",$audited,'-o',"$dir/count.exe")
    Native 'clang++' @("$Out/timing.obj",$normal,'-o',"$dir/step.exe")
    foreach ($mode in @('memory','prefill')) {
        Native "$dir/count.exe" @($mode) | Set-Content -Encoding ascii "$dir/$mode.csv"
    }
    $rows = @(& "$dir/step.exe" shared8)
    # Source intentionally misses five physical tolerances. Compare the complete, same native outputs.
    if ($LASTEXITCODE -notin @(0,1) -or $rows.Count -ne 37) { throw "Incomplete sweep: $v" }
    $rows | Set-Content -Encoding ascii "$dir/grip.csv"
    $rows = @(Native "$dir/step.exe" @('trace-order'))
    if ($rows.Count -ne 721) { throw "Incomplete impulse trace: $v" }
    $rows | Set-Content -Encoding ascii "$dir/jaws.csv"
}
foreach ($v in @('reuse','heap')) {
    foreach ($csv in @('grip.csv','jaws.csv')) {
        if ((Get-FileHash "$Out/$v/$csv").Hash -ne (Get-FileHash "$Out/stock/$csv").Hash) { throw "Output changed: $v/$csv" }
    }
}
$env:KP_GRIP_TIMING = '1'
$orders = @('012','120','201','210','102','021')
'block,slot,variant,frames,us_per_frame,end_drift_mm' | Set-Content -Encoding ascii "$Out/timing.csv"
for ($block = 0; $block -lt 18; ++$block) {
    $slot = 0
    foreach ($digit in $orders[$block % 6].ToCharArray()) {
        $v = $variants[[int]::Parse([string]$digit)]
        $rows = @(Native "$Out/$v/step.exe" @('shared8'))
        if ($rows.Count -ne 2) { throw "Timing failed: $v" }
        $fields = $rows[1].Split(',')
        "$block,$slot,$v,$($fields[1]),$($fields[2]),$($fields[3])" | Add-Content -Encoding ascii "$Out/timing.csv"
        ++$slot
    }
}
Get-FileHash ($variants | ForEach-Object { "$Out/$_/step.exe"; "$Out/$_/count.exe" }) |
    Select-Object @{Name='binary';Expression={ (Split-Path -Leaf (Split-Path $_.Path)) + '/' + (Split-Path -Leaf $_.Path) }},Hash |
    ConvertTo-Csv -NoTypeInformation | Set-Content -Encoding ascii "$Out/binaries.csv"
