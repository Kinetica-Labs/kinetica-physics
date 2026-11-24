# Native Windows timings. Each configuration occupies every slot twice.
param([Parameter(Mandatory=$true)][string]$Trial, [Parameter(Mandatory=$true)][string]$OutputDir)
$ErrorActionPreference = 'Stop'
$trialPath = (Resolve-Path $Trial).Path
$out = (Resolve-Path $OutputDir).Path
$kp = "$trialPath/optin-build/validation/studies/study-grip-limit.exe"
$programs = [ordered]@{
    'kinetica-off'=$kp; 'kinetica-on'=$kp; box3d="$trialPath/box3d.exe"
    'rp3d-f32'="$trialPath/rp3d-float.exe"; 'rp3d-f64'="$trialPath/rp3d-double.exe"
    bullet="$out/bullet.exe"; jolt="$out/jolt.exe"
}
$process = Get-Process -Id $PID
$prior = $process.ProcessorAffinity
$names = @($programs.Keys)
try {
    $process.ProcessorAffinity = [IntPtr]0x100
    $env:KP_THREADS = '1'
    $env:KP_GRIP_PROFILE = '0'
    foreach ($name in $names) {
        $env:KP_GRIP_TIMING = '0'
        $env:KP_COUPLED_FRICTION = if ($name -eq 'kinetica-on') { '1' } else { '0' }
        $dir = New-Item -ItemType Directory -Force "$out/$name"
        $exe = $programs[$name]
        $sweep = @(& $exe shared8)
        if ($LASTEXITCODE -notin @(0,1) -or $sweep.Count -ne 37) { throw "Incomplete sweep $name" }
        $sweep | Set-Content -Encoding ascii "$dir/grip.csv"
        (Get-FileHash $exe).Hash | Set-Content -Encoding ascii "$dir/timing-sha256.txt"
    }
    $env:KP_GRIP_TIMING = '1'
    $n = $names.Length
    $records = for ($block = 0; $block -lt 2*$n; $block++) {
        for ($slot = 0; $slot -lt $n; $slot++) {
            $offset = if ($block -lt $n) { $slot } else { $n - 1 - $slot }
            $name = $names[($block + $offset) % $n]
            $env:KP_COUPLED_FRICTION = if ($name -eq 'kinetica-on') { '1' } else { '0' }
            $exe = $programs[$name]
            $run = @(& $exe shared8)
            if ($LASTEXITCODE -ne 0 -or $run.Count -ne 2) { throw "Incomplete timing $name" }
            $r = $run | ConvertFrom-Csv
            if ($r.frames -ne '360' -or [double]$r.us_per_frame -le 0) { throw "Invalid timing $name" }
            [pscustomobject]@{ block=$block;slot=$slot;solver=$name;frames=$r.frames;us_per_frame=$r.us_per_frame;end_drift_mm=$r.end_drift_mm }
        }
    }
    $records | Export-Csv -NoTypeInformation -Encoding UTF8 "$out/native-timing.csv"
} finally {
    $process.ProcessorAffinity = $prior
}
