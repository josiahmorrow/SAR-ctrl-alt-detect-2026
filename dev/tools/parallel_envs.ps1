<#
Run one batch per world at the same time, each in its own copy of the
environment, so an experiment costs one mission's wall time instead of three.

  powershell -File dev\tools\parallel_envs.ps1 -Tag exp01 -MissionS 600 [-LargeRuns 1] [-DebugVision]

Each world runs in sar-env (small), sar-env2 (medium), sar-env3 (large). The
copies are synced from sar-env before every launch: the organisers' files
(worlds, recordings, protos, supervisor), the dev supervisor/world copies,
the flyover snapshot cache, and this repo's working tree. Results from all
three land in the main repo's dev/eval/runs/<tag>.csv (SAR_OUT_DIR).
Missions are independent and deterministic, so running them side by side
does not change their outcome; only wall-clock time.
#>
param(
    [Parameter(Mandatory = $true)][string]$Tag,
    [int]$MissionS = 180,
    [int]$LargeRuns = 1,
    [switch]$DebugVision,
    [string]$Worlds = "small,medium,large"
)
$ErrorActionPreference = "Stop"
$main = (Resolve-Path "$PSScriptRoot\..\..\..\..").Path          # sar-env
$parent = Split-Path $main -Parent
$py = Join-Path $main ".venv\Scripts\python.exe"
$sol = Join-Path $main "controllers\proposed_solution"
$out = Join-Path $sol "dev\eval\runs"
$envFor = @{ small = $main; medium = (Join-Path $parent "sar-env2"); large = (Join-Path $parent "sar-env3") }

foreach ($w in $Worlds.Split(",")) {
    $dst = $envFor[$w]
    if ($dst -ne $main) {
        # Organisers' tree + dev copies + snapshot cache (no venv, no logs).
        robocopy $main $dst /MIR /XD "$main\.venv" "$main\log" "$main\.git" "$main\controllers\proposed_solution" /XF "batch_*.out" "batch_*.err" /NFL /NDL /NJH /NJS /NP | Out-Null
        # This repo's working tree, minus run outputs and git metadata.
        robocopy $sol (Join-Path $dst "controllers\proposed_solution") /MIR /XD "$sol\.git" "$sol\dev\eval" "$sol\sim_logs" "$sol\__pycache__" /NFL /NDL /NJH /NJS /NP | Out-Null
        New-Item -ItemType Directory -Force (Join-Path $dst "controllers\proposed_solution\sim_logs") | Out-Null
        New-Item -ItemType Directory -Force (Join-Path $dst "log") | Out-Null
    }
}

$i = 0
foreach ($w in $Worlds.Split(",")) {
    $dst = $envFor[$w]
    $runs = if ($w -eq "large") { $LargeRuns } else { 1 }
    $batchArgs = @("-u", "-X", "utf8", "controllers\proposed_solution\dev\tools\run_batch.py",
              "--world", $w, "--runs", "$runs", "--tag", $Tag, "--mission-s", "$MissionS")
    if ($DebugVision) { $batchArgs += "--debug-vision" }
    $env:SAR_OUT_DIR = $out
    $env:SAR_PARALLEL = "1"
    Start-Process -WindowStyle Hidden -FilePath $py -ArgumentList $batchArgs -WorkingDirectory $dst `
        -RedirectStandardOutput (Join-Path $main "_eval\batch_${Tag}_$w.out") `
        -RedirectStandardError (Join-Path $main "_eval\batch_${Tag}_$w.err")
    $i++
    Start-Sleep -Seconds 8          # stagger: first batch creates the CSV header
}
"launched $i batches for tag $Tag at $(Get-Date -Format 'HH:mm:ss')"

