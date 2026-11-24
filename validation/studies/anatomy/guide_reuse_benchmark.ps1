# Paired comparison on native Windows. Builds and binaries are read-only.
param(
    [Parameter(Mandatory=$true)][string]$BaselineBuild,
    [Parameter(Mandatory=$true)][string]$CandidateBuild,
    [Parameter(Mandatory=$true)][string]$OutputDir,
    [int]$ProcessorMask=0x100,
    [ValidateSet('matrix','factor','factor-simple','normal')][string]$Change='matrix'
)
$ErrorActionPreference='Stop'
[Threading.Thread]::CurrentThread.CurrentCulture=[Globalization.CultureInfo]::InvariantCulture
$builds=[ordered]@{ A=(Resolve-Path $BaselineBuild).Path; B=(Resolve-Path $CandidateBuild).Path }
if ($builds.A -eq $builds.B) { throw 'Two independent builds are required' }
if ((Test-Path $OutputDir) -and @(Get-ChildItem $OutputDir -Force).Count) { throw "Output directory is not empty: $OutputDir" }
$out=(New-Item -ItemType Directory -Force $OutputDir).FullName
$process=Get-Process -Id $PID
$oldMask=$process.ProcessorAffinity
$oldEnv=@{}
foreach ($e in 'KP_THREADS','KP_COUPLED_FRICTION','KP_GRIP_TIMING','KP_GRIP_PROFILE','BENCH') {
    $oldEnv[$e]=[Environment]::GetEnvironmentVariable($e)
}
function Median([double[]]$values) {
    $s=@($values | Sort-Object)
    if (!$s.Length) { throw 'Empty sample' }
    $mid=[int][Math]::Floor($s.Length / 2)
    if ($s.Length % 2) { return $s[$mid] }
    return ($s[$mid-1]+$s[$mid])/2
}

try {
    $process.ProcessorAffinity=[IntPtr]$ProcessorMask
    $env:KP_THREADS='1'
    $env:KP_GRIP_PROFILE='0'
    $env:KP_GRIP_TIMING='0'
    $env:BENCH=$null
    $meta=[ordered]@{ utc_start=[DateTime]::UtcNow.ToString('o'); system=$env:COMPUTERNAME
        cpu=(Get-CimInstance Win32_Processor | Select-Object -First 1 -ExpandProperty Name)
        os=[Environment]::OSVersion.VersionString; inherited_affinity_mask=$process.ProcessorAffinity.ToInt64()
        worker_threads=1; A='baseline'; B='candidate'; tested_change=$Change; design='successive AB and BA, 10 pairs of each order per scene'
        grip='shared8, coupled enabled, 8 public steps at 1/480 s per frame; mu .5, squeeze 10 N per jaw, vertical load 10 N; 15 preload frames, 15 loaded frames, 3 warmup and 24 timed rigs per sample'
        pyramid='BENCH=7 compare-ours realtime 8; 140 free cubes, 60 frames, 8 internal substeps, sleep off, default (uncoupled) solver; reported to 0.01 ms'
    }
    $exe=@{}
    $sources=@{}
    $exeRows=foreach ($v in $builds.Keys) {
        $root=$builds[$v]
        $srcLine=Get-Content "$root/CMakeCache.txt" | Where-Object {$_ -like 'CMAKE_HOME_DIRECTORY:INTERNAL=*'}
        if (!$srcLine) { throw "Missing source root in $root" }
        $src=(Resolve-Path $srcLine.Substring($srcLine.IndexOf('=')+1)).Path
        $sources[$v]=$src
        $exe[$v]=@{grip="$root/validation/studies/study-grip-limit.exe"; pyramid="$root/validation/engines/compare-ours.exe"}
        foreach ($scene in 'grip','pyramid') {
            $file=Get-Item $exe[$v][$scene]
            [pscustomobject]@{variant=$v;scene=$scene;path=$file.FullName;sha256=(Get-FileHash $file.FullName).Hash;source_root=$src}
        }
        Get-Content "$root/CMakeCache.txt" | Where-Object {$_ -match '^CMAKE_(CXX_COMPILER|BUILD_TYPE|CXX_FLAGS)(:|_)'} |
            Set-Content "$out/compiler-$v.txt" -Encoding ASCII
        Get-ChildItem "$root/src" -Recurse -Filter flags.make | Where-Object {$_.FullName -like '*kinetica-physics.dir*'} |
            ForEach-Object {Get-Content $_.FullName} | Add-Content "$out/compiler-$v.txt" -Encoding ASCII
    }
    $exeRows | Export-Csv "$out/executables.csv" -NoTypeInformation -Encoding ASCII
    $tables=@{}
    $sourceRows=foreach ($v in $builds.Keys) {
        $root=$sources[$v]
        $table=@{}
        $kernel=Get-ChildItem "$root/include","$root/src" -Recurse -File
        $harness=Get-Item "$root/validation/studies/GripLimit.cpp","$root/validation/studies/GripLimit.h",
                             "$root/validation/studies/StudyFriction.h","$root/validation/engines/common.h","$root/validation/engines/ours.cpp"
        foreach ($f in (@($kernel)+@($harness))) {
            $relative=$f.FullName.Substring($root.Length+1).Replace('\','/')
            $hash=(Get-FileHash $f.FullName).Hash
            $table[$relative]=$hash
            [pscustomobject]@{variant=$v;path=$relative;sha256=$hash}
        }
        $tables[$v]=$table
    }
    $sourceRows | Sort-Object path,variant | Export-Csv "$out/source-hashes.csv" -NoTypeInformation -Encoding ASCII
    $allKeys=@( (@($tables.A.Keys) + @($tables.B.Keys)) | Sort-Object -Unique)
    $changed=@($allKeys | Where-Object {$tables.A[$_] -ne $tables.B[$_]})
    $changed | Set-Content "$out/source-differences.txt" -Encoding ASCII
    $expected=@('include/dynamics/time_stepping/DirectJoints.h','src/dynamics/time_stepping/FrictionIslands.cpp')
    if ($Change -like 'factor*') {
        $expected+=@('include/math/ProfileMatrix.h','src/dynamics/time_stepping/DirectJoints.cpp',
                     'src/dynamics/time_stepping/CoupledFriction.cpp')
        if ($Change -eq 'factor') { $expected+='include/dynamics/time_stepping/CoupledFriction.h' }
    }
    if ($Change -eq 'normal') { $expected=@('src/dynamics/event_driven/ContactSystem.cpp') }
    if (@(Compare-Object $expected $changed).Count) { throw "Different source changes; see source-differences.txt" }
    $parity=foreach ($mode in '0','1') {
        $env:KP_COUPLED_FRICTION=$mode
        $hashes=@{}
        foreach ($v in $builds.Keys) {
            $p=$exe[$v].grip
            $lines=@(& $p shared8)
            $status=$LASTEXITCODE
            $file="$out/grip-coupled-$mode-$v.csv"
            $lines | Set-Content $file -Encoding ASCII
            $rows=@($lines | ConvertFrom-Csv)
            $failed=@($rows | Where-Object {$_.pass -eq '0'}).Count
            if ($rows.Count -ne 36 -or $status -ne [int]($failed -ne 0) -or
                @($rows | Where-Object {$_.complete -ne '1' -or $_.engine -ne 'kinetica-shared8'}).Count) { throw "Incomplete sweep: $file" }
            $hashes[$v]=(Get-FileHash $file).Hash
            [pscustomobject]@{coupled=$mode;variant=$v;rows=$rows.Count;pass=$rows.Count-$failed;sha256=$hashes[$v]}
        }
        if ($hashes.A -ne $hashes.B) { throw "Changed physical grip observations in mode $mode" }
    }
    $parity | Export-Csv "$out/parity.csv" -NoTypeInformation -Encoding ASCII
    $timings=foreach ($scene in 'grip','pyramid') {
        if ($scene -eq 'grip') {$env:KP_GRIP_TIMING='1'; $env:KP_COUPLED_FRICTION='1'; $env:BENCH=$null}
        else {$env:KP_GRIP_TIMING='0'; $env:KP_COUPLED_FRICTION='0'; $env:BENCH='7'}
        $drift=$null
        for ($i=1;$i -le 20;$i++) {
            $order=if ($i % 2) {'AB'} else {'BA'}
            for ($j=0;$j -lt 2;$j++) {
                $v=$order.Substring($j,1)
                $p=$exe[$v][$scene]
                $worst=$null
                if ($scene -eq 'grip') {
                    $lines=@(& $p shared8)
                    if ($LASTEXITCODE -ne 0 -or $lines.Count -ne 2) { throw "Grip timer failed: $i,$v" }
                    $r=$lines | ConvertFrom-Csv
                    if ($r.engine -ne 'kinetica-shared8' -or $r.frames -ne '360') { throw 'Wrong grip workload' }
                    $us=[double]$r.us_per_frame; $frames=[int]$r.frames
                    if ($null -eq $drift) {$drift=$r.end_drift_mm}
                    if ($drift -ne $r.end_drift_mm) { throw 'Timed grip drift changed' }
                } else {
                    $lines=@(& $p realtime 8)
                    if ($LASTEXITCODE -ne 0 -or $lines.Count -ne 1 -or $lines[0] -notmatch '140:\s*([\d.]+)\s+.*?([\d.]+)\s*$') {
                        throw "Wrong pyramid workload: $lines"
                    }
                    $us=1000*[double]$Matches[1]; $worst=1000*[double]$Matches[2]; $frames=60
                }
                if (!($us -gt 0) -or [double]::IsInfinity($us)) { throw "Invalid timing: $scene/$i/$v" }
                [pscustomobject]@{scene=$scene;pair=$i;slot=$j+1;order=$order;variant=$v;frames=$frames;us_per_frame=$us;max_frame_us=$worst;end_drift_mm=$drift}
            }
        }
    }
    $timings | Export-Csv "$out/timing.csv" -NoTypeInformation -Encoding ASCII
    $pairs=foreach ($scene in 'grip','pyramid') {
        foreach ($i in 1..20) {
            $pair=@($timings | Where-Object {$_.scene -eq $scene -and $_.pair -eq $i})
            $a=$pair | Where-Object {$_.variant -eq 'A'}
            $b=$pair | Where-Object {$_.variant -eq 'B'}
            [pscustomobject]@{scene=$scene;pair=$i;order=$a.order;baseline_us=$a.us_per_frame;candidate_us=$b.us_per_frame;candidate_over_baseline=$b.us_per_frame/$a.us_per_frame}
        }
    }
    $pairs | Export-Csv "$out/paired.csv" -NoTypeInformation -Encoding ASCII
    $summary=foreach ($scene in 'grip','pyramid') {
        $rows=@($pairs | Where-Object {$_.scene -eq $scene})
        $ab=@($rows | Where-Object {$_.order -eq 'AB'}); $ba=@($rows | Where-Object {$_.order -eq 'BA'})
        [pscustomobject]@{scene=$scene;pairs=$rows.Count;baseline_median_us=(Median $rows.baseline_us);candidate_median_us=(Median $rows.candidate_us)
            median_candidate_over_baseline=(Median $rows.candidate_over_baseline);ab_median_ratio=(Median $ab.candidate_over_baseline);ba_median_ratio=(Median $ba.candidate_over_baseline)
            candidate_faster=@($rows | Where-Object {$_.candidate_over_baseline -lt 1}).Count;ties=@($rows | Where-Object {$_.candidate_over_baseline -eq 1}).Count}
    }
    $summary | Export-Csv "$out/summary.csv" -NoTypeInformation -Encoding ASCII
    $meta['utc_end']=[DateTime]::UtcNow.ToString('o')
    $meta | ConvertTo-Json | Set-Content "$out/method.json" -Encoding ASCII
    $summary | Format-Table
} finally {
    $process.ProcessorAffinity=$oldMask
    foreach ($e in $oldEnv.Keys) {[Environment]::SetEnvironmentVariable($e,$oldEnv[$e])}
}
