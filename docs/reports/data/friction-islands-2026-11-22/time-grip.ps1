# Compare the same loaded grip with 8 public updates per frame on one Windows CPU.
param([Parameter(Mandatory=$true)][string]$Lab, [Parameter(Mandatory=$true)][string]$Output)
$ErrorActionPreference = 'Stop'
$trial = (Resolve-Path $Lab).Path
$programs = [ordered]@{
    baseline = "$trial/bin/baseline/study-grip-limit.exe"
    disks = "$trial/bin/disks/study-grip-limit.exe"
    cache = "$trial/bin/cache/study-grip-limit.exe"
    box3d = "$trial/box3d.exe"
    'rp3d-float' = "$trial/rp3d-float.exe"
    'rp3d-double' = "$trial/rp3d-double.exe"
}
$process = Get-Process -Id $PID
$affinity = $process.ProcessorAffinity
$oldThreads, $oldTiming, $oldProfile = $env:KP_THREADS, $env:KP_GRIP_TIMING, $env:KP_GRIP_PROFILE
try {
    $process.ProcessorAffinity = [IntPtr]0x100
    $env:KP_THREADS = '1'
    $env:KP_GRIP_TIMING = '1'
    $env:KP_GRIP_PROFILE = '0'
    $names = @($programs.Keys)
    # Six cyclic orders, then their reversals. Each engine occupies every slot twice.
    $records = for ($block = 0; $block -lt 12; $block++) {
        for ($slot = 0; $slot -lt 6; $slot++) {
            $i = ($block + $(if ($block -lt 6) { $slot } else { 5 - $slot })) % 6
            $name = $names[$i]
            $program = $programs[$name]
            $lines = @(& $program shared8)
            if ($LASTEXITCODE -ne 0 -or $lines.Count -ne 2) { throw "Incomplete timing $name $block" }
            $r = $lines | ConvertFrom-Csv
            if ($r.frames -ne '360') { throw "Unexpected frame count $name" }
            [pscustomobject]@{ block=$block; slot=$slot; solver=$name; frames=$r.frames
                us_per_frame=$r.us_per_frame; end_drift_mm=$r.end_drift_mm }
        }
    }
    $records | Export-Csv -NoTypeInformation -Encoding UTF8 $Output
} finally {
    $process.ProcessorAffinity = $affinity
    $env:KP_THREADS, $env:KP_GRIP_TIMING, $env:KP_GRIP_PROFILE = $oldThreads, $oldTiming, $oldProfile
}
