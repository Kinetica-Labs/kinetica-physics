# Compile observers against the exact Windows libraries used for timing.
param([string]$Base, [string]$OutputDir, [string]$KineticaBuild, [string]$KineticaSource)
$ErrorActionPreference = 'Stop'
$out = (Resolve-Path $OutputDir).Path
$mem = (New-Item -Force -ItemType Directory "$out/memory").FullName
$here = "validation/studies/anatomy"
$release = @('-O3','-DNDEBUG','-std=c++17','-ffp-contract=off')
function Compile([string[]]$Options) {
    & clang++ @Options
    if ($LASTEXITCODE) { throw "compile failed: $Options" }
}
$bullet = (Resolve-Path build/anatomy-thirdparty/bullet/src).Path
$bl = "$out/bullet-build/lib"
Compile ($release + @("-I$Base/box3d/include","$here/box_memory.cpp","$Base/build-box3d/src/libbox3d.a",'-o',"$mem/box3d.exe"))
Compile ($release + @("-I$bullet","$here/bullet_memory.cpp","$bl/libBulletDynamics.a","$bl/libBulletCollision.a","$bl/libLinearMath.a",'-o',"$mem/bullet.exe"))
foreach ($precision in @('float','double')) {
    $extra = @()
    if ($precision -eq 'double') { $extra += '-DIS_RP3D_DOUBLE_PRECISION_ENABLED' }
    Compile ($release + $extra + @("-I$Base/rp3d/include","$here/rp_memory.cpp","$Base/build-rp3d-$precision/libreactphysics3d.a",'-o',"$mem/rp3d-$precision.exe"))
}
$src = (Resolve-Path build/anatomy-thirdparty/jolt).Path
$settings = Get-Content "$out/jolt-build/CMakeFiles/Jolt.dir/flags.make"
$defines = @(($settings | Select-String '^CXX_DEFINES =').Line.Substring(14).Trim() -split ' ')
$flags = @(($settings | Select-String '^CXX_FLAGS =').Line.Substring(12).Trim() -split ' ')
Compile ($flags + $defines + @("-I$src","$here/jolt_memory.cpp","$out/jolt-build/libJolt.a",'-o',"$mem/jolt.exe"))
Compile ($release + @("-I$KineticaSource/include","$here/kinetica_memory.cpp",'validation/studies/memory/MemoryCounters.cpp',"$KineticaBuild/src/libkinetica-physics.a",'-o',"$mem/kinetica.exe"))
$env:KP_THREADS = '1'
foreach ($name in @('box3d','bullet','rp3d-float','rp3d-double','jolt','kinetica-stock','kinetica-coupled')) {
    $stem = $name
    $options = ''
    if ($name.StartsWith('kinetica-')) { $stem = 'kinetica'; $options = $name.Substring('kinetica-'.Length) }
    $processOptions = @{FilePath="$mem/$stem.exe";NoNewWindow=$true;PassThru=$true;Wait=$true;
        RedirectStandardOutput="$mem/$name.csv";RedirectStandardError="$mem/$name-trace.txt"}
    if ($options) { $processOptions.ArgumentList = @($options) }
    $p = Start-Process @processOptions
    if ($p.ExitCode -ne 0) { throw "probe failed: $name $($p.ExitCode)" }
    (Get-FileHash "$mem/$stem.exe").Hash | Set-Content -Encoding ascii "$mem/$name-sha256.txt"
}
