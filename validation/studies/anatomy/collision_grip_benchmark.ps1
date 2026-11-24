# Windows PowerShell 5.1. Pass a fresh directory to preserve previous raw samples.
param(
    [Parameter(Mandatory=$true)][string]$Kinetica,
    [Parameter(Mandatory=$true)][string]$Box3D,
    [Parameter(Mandatory=$true)][string]$ReactPhysics3D,
    [Parameter(Mandatory=$true)][string]$OutputDir,
    [int]$ProcessorMask = 0x100
)
$ErrorActionPreference = 'Stop'
[Threading.Thread]::CurrentThread.CurrentCulture = [Globalization.CultureInfo]::InvariantCulture
$process = Get-Process -Id $PID
$previousMask = $process.ProcessorAffinity
$programs = [ordered]@{
    kinetica = (Resolve-Path $Kinetica).Path
    box3d = (Resolve-Path $Box3D).Path
    rp3d = (Resolve-Path $ReactPhysics3D).Path
}
$labels = @{kinetica='kinetica-shared8-coupled';box3d='box3d-shared8';rp3d='rp3d-f64-shared8'}
$precisions = @{kinetica='double';box3d='float';rp3d='double'}
if ((Test-Path $OutputDir) -and @(Get-ChildItem -Force $OutputDir).Count) {
    throw "Output directory is not empty: $OutputDir"
}
$out = (New-Item -ItemType Directory -Force $OutputDir).FullName
$envNames = @('KP_THREADS','KP_GRIP_PROFILE','KP_GRIP_TIMING','KP_COUPLED_FRICTION')
$previousEnv = @{}
foreach ($envName in $envNames) { $previousEnv[$envName] = [Environment]::GetEnvironmentVariable($envName) }
function Get-Median([double[]]$Values) {
    $s = @($Values | Sort-Object)
    if (!$s.Count) { throw 'Empty sample' }
    $middle = [int][Math]::Floor($s.Count / 2)
    if ($s.Count % 2) { return $s[$middle] }
    return ($s[$middle-1] + $s[$middle]) / 2
}
function Get-CaseKey($row) {
    return ($row.hz,$row.mu,$row.jaw_N,$row.load_N,$row.open -join '|')
}
try {
    $process.ProcessorAffinity = [IntPtr]$ProcessorMask
    $env:KP_THREADS = '1'
    $env:KP_GRIP_PROFILE = '0'
    $env:KP_GRIP_TIMING = '0'
    $manifest = foreach ($name in $programs.Keys) {
        $exe = $programs[$name]
        [pscustomobject]@{ solver=$name; precision=$precisions[$name]; sha256=(Get-FileHash $exe).Hash; exe=$exe }
    }
    $manifest | Export-Csv "$out/executables.csv" -NoTypeInformation -Encoding ASCII
    $meta = [ordered]@{
        utc_start=[DateTime]::UtcNow.ToString('o'); host_name=$env:COMPUTERNAME
        os=[Environment]::OSVersion.VersionString; cpu=(Get-CimInstance Win32_Processor | Select-Object -First 1 -ExpandProperty Name)
        parent_affinity_mask=$process.ProcessorAffinity.ToInt64(); worker_threads=1; public_steps_per_frame=8
        timer_hz=60; timer_friction=0.5; timer_squeeze_per_jaw_N=10; timer_load_N=10
        preload_frames=15; timed_frames_per_fixture=15; warmup_fixtures=3; measured_fixtures_per_process=24
        physical_time_per_phase_s=0.25; timing_blocks=12; native_mode='shared8'; timing_scope='loaded step loop only'
        solver_config=@{kinetica='coupled friction on, one realtime substep and contact update per call';box3d='one substep per call, native warm start enabled';rp3d='double, one velocity and one position iteration per call'}
        accuracy='abs(path - oracle) in mm; force gate 0.2 N; hold 0.5 mm; slide 2 mm'
        ordering='all six permutations twice, common block for pairwise ratios'
    }
    $sweeps=@{}
    foreach ($name in $programs.Keys) {
        $env:KP_COUPLED_FRICTION = if ($name -eq 'kinetica') {'1'} else {'0'}
        $exe=$programs[$name]
        $lines=@(& $exe shared8)
        $status=$LASTEXITCODE
        if ($lines.Count -ne 37 -or $status -notin @(0,1)) { throw "Incomplete physical sweep: $name ($status)" }
        $lines | Set-Content "$out/$name-grip.csv" -Encoding ASCII
        $byKey=@{}
        foreach ($r in ($lines | ConvertFrom-Csv)) {
            $key=Get-CaseKey $r
            if ($r.engine -ne $labels[$name] -or $r.complete -ne '1' -or $byKey.ContainsKey($key)) {
                throw "Duplicate, incomplete or unexpected physical record: $name/$key"
            }
            $byKey[$key]=$r
        }
        $sweeps[$name]=$byKey
    }
    $env:KP_GRIP_TIMING='1'
    $names=@($programs.Keys)
    $timings = for ($block=0; $block -lt 12; $block++) {
        for ($slot=0; $slot -lt 3; $slot++) {
            $reverse=[int][Math]::Floor($block / 3) % 2
            $offset=if ($reverse) { 2-$slot } else { $slot }
            $name=$names[($block+$offset) % 3]
            $env:KP_COUPLED_FRICTION = if ($name -eq 'kinetica') {'1'} else {'0'}
            $exe=$programs[$name]
            $lines=@(& $exe shared8)
            if ($LASTEXITCODE -ne 0 -or $lines.Count -ne 2) { throw "Incomplete timing: $block/$name" }
            $r=$lines | ConvertFrom-Csv
            $us=[double]$r.us_per_frame
            if ($r.engine -ne $labels[$name] -or $r.frames -ne '360' -or !($us -gt 0) -or [double]::IsInfinity($us)) {
                throw "Invalid timing record: $block/$name"
            }
            [pscustomobject]@{ block=$block; slot=$slot; solver=$name; engine=$r.engine; frames=$r.frames; us_per_frame=$r.us_per_frame; end_drift_mm=$r.end_drift_mm }
        }
    }
    $timings | Export-Csv "$out/timing.csv" -NoTypeInformation -Encoding ASCII
    $ratioRows = foreach ($block in 0..11) {
        $group=@($timings | Where-Object { $_.block -eq $block })
        $k=$group | Where-Object { $_.solver -eq 'kinetica' }
        foreach ($other in 'box3d','rp3d') {
            $p=$group | Where-Object { $_.solver -eq $other }
            [pscustomobject]@{ block=$block; peer=$other; kinetica_us=[double]$k.us_per_frame; peer_us=[double]$p.us_per_frame
                kinetica_over_peer=[double]$k.us_per_frame / [double]$p.us_per_frame }
        }
    }
    $ratioRows | Export-Csv "$out/paired-time.csv" -NoTypeInformation -Encoding ASCII
    $accuracy = foreach ($other in 'box3d','rp3d') {
        foreach ($key in ($sweeps['kinetica'].Keys | Sort-Object)) {
            $k=$sweeps['kinetica'][$key]; $p=$sweeps[$other][$key]
            if (!$p -or $k.expected_drop_mm -ne $p.expected_drop_mm) { throw "Different oracle: $other/$key" }
            $forceK=[Math]::Max([double]$k.max_left_force_error_N,[double]$k.max_right_force_error_N)
            $forceP=[Math]::Max([double]$p.max_left_force_error_N,[double]$p.max_right_force_error_N)
            [pscustomobject]@{ peer=$other; hz=$k.hz; mu=$k.mu; jaw_N=$k.jaw_N; load_N=$k.load_N; open=$k.open
                kinetica_max_path_error_mm=$k.max_path_error_mm; peer_max_path_error_mm=$p.max_path_error_mm
                delta_error_mm=[double]$k.max_path_error_mm-[double]$p.max_path_error_mm
                both_force_gates_pass=($forceK -le .2 -and $forceP -le .2); kinetica_pass=$k.pass; peer_pass=$p.pass }
        }
    }
    $accuracy | Export-Csv "$out/paired-accuracy.csv" -NoTypeInformation -Encoding ASCII
    $summary=foreach ($other in 'box3d','rp3d') {
        $pairs=@($ratioRows | Where-Object { $_.peer -eq $other })
        $cases=@($accuracy | Where-Object { $_.peer -eq $other })
        $forceValid=@($cases | Where-Object { $_.both_force_gates_pass })
        $timed=@($cases | Where-Object {
            [int]$_.hz -eq $meta.timer_hz -and [double]$_.mu -eq $meta.timer_friction -and
            [double]$_.jaw_N -eq $meta.timer_squeeze_per_jaw_N -and [double]$_.load_N -eq $meta.timer_load_N -and [int]$_.open -eq 0
        })
        if ($timed.Count -ne 1) { throw "Missing or ambiguous timed case: $other" }
        $rawRatio=Get-Median $pairs.kinetica_over_peer
        $qualified=$timed[0].kinetica_pass -eq '1' -and $timed[0].peer_pass -eq '1'
        $tieMm=1e-6
        [pscustomobject]@{ peer=$other; blocks=$pairs.Count; median_kinetica_over_peer=$rawRatio
            timed_kinetica_pass=$timed[0].kinetica_pass; timed_peer_pass=$timed[0].peer_pass
            median_at_tolerance=$(if ($qualified) { $rawRatio })
            blocks_kinetica_faster=@($pairs | Where-Object { $_.kinetica_over_peer -lt 1 }).Count
            all_cases=$cases.Count; kinetica_pass=@($cases | Where-Object { $_.kinetica_pass -eq '1' }).Count
            peer_pass=@($cases | Where-Object { $_.peer_pass -eq '1' }).Count; force_valid_pairs=$forceValid.Count
            tie_mm=$tieMm; smaller_path_error_kinetica=@($forceValid | Where-Object { $_.delta_error_mm -lt -$tieMm }).Count
            tied_path_error=@($forceValid | Where-Object { [Math]::Abs($_.delta_error_mm) -le $tieMm }).Count
            smaller_path_error_peer=@($forceValid | Where-Object { $_.delta_error_mm -gt $tieMm }).Count }
    }
    $summary | Export-Csv "$out/summary.csv" -NoTypeInformation -Encoding ASCII
    $meta['utc_end']=[DateTime]::UtcNow.ToString('o')
    $meta | ConvertTo-Json -Depth 3 | Set-Content "$out/method.json" -Encoding ASCII
    $summary | Format-Table
} finally {
    $process.ProcessorAffinity=$previousMask
    foreach ($envName in $envNames) { [Environment]::SetEnvironmentVariable($envName, $previousEnv[$envName]) }
}
