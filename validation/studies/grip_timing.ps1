# Запуск из каталога стенда. Пути внешних адаптеров приведены в отчёте grip-limit.md.
param([long] $Affinity = 0x100, [switch] $SharedClock)
$ErrorActionPreference = 'Stop'
$process = [System.Diagnostics.Process]::GetCurrentProcess()
$oldAffinity = $process.ProcessorAffinity
$oldTiming = $env:KP_GRIP_TIMING
$oldThreads = $env:KP_THREADS
$kp = '.\build\validation\studies\study-grip-limit.exe'
$programs = @($kp, $kp, $kp, '.\grip-box3d.exe', '.\grip-rp3d.exe', '.\grip-rp3d-f64.exe')
$modes = @('exact', 'realtime', 'realtime32')
$offsets = @(0,1,5,2,4,3)
if ($SharedClock) {
    $programs = @($kp, '.\grip-box3d.exe', '.\grip-rp3d.exe', '.\grip-rp3d-f64.exe')
    $modes = @('shared8', 'shared8', 'shared8', 'shared8')
    $offsets = @(0,1,3,2)
}
try {
    $process.ProcessorAffinity = [IntPtr]$Affinity
    $env:KP_THREADS = '1'
    $env:KP_GRIP_TIMING = '1'
    'block,slot,engine,frames,us_per_frame,end_drift_mm'
    for ($block = 0; $block -lt 12; ++$block) {
        $slot = 0
        foreach ($offset in $offsets) {
            $i = ($block + $offset) % $programs.Count
            $arguments = @()
            if ($i -lt $modes.Count) { $arguments = @($modes[$i]) }
            $program = $programs[$i]
            $lines = @(& $program @arguments)
            if ($LASTEXITCODE -ne 0 -or $lines.Count -ne 2) { throw "Failed: $program $arguments" }
            "$block,$slot,$($lines[1])"
            ++$slot
        }
    }
} finally {
    $process.ProcessorAffinity = $oldAffinity
    $env:KP_GRIP_TIMING = $oldTiming
    $env:KP_THREADS = $oldThreads
}
