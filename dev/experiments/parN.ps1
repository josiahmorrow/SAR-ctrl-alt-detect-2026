param([string]$Jobs, [int]$Slots = 3, [int]$MissionS = 600)
# Job scheduler over parallel environment copies (sar-env, sar-env2, sar-env3).
# $Jobs: text file, one job per line: "<exp> <controller.py> <world> <seed>".
# Each job runs alone in a free env with that controller; tag <exp>_<W><seed>.
$root = "$env:USERPROFILE\projects\sar"
$main = "$root\sar-env"; $sol = "$main\controllers\proposed_solution"
$py   = "$main\.venv\Scripts\python.exe"; $o = "$main\_eval"
$envs = @($main, "$root\sar-env2", "$root\sar-env3")[0..($Slots - 1)]
while (Get-CimInstance Win32_Process -Filter "Name like 'python%'" | Where-Object { $_.CommandLine -match 'run_batch' }) { Start-Sleep 15 }
foreach ($e in $envs) {
    if ($e -eq $main) { continue }
    robocopy $main $e /MIR /XD "$main\.venv" "$main\log" "$main\.git" "$sol" /XF "batch_*.out" "batch_*.err" "q_*" /NFL /NDL /NJH /NJS /NP | Out-Null
    robocopy $sol "$e\controllers\proposed_solution" /MIR /XD "$sol\.git" "$sol\dev\eval" "$sol\sim_logs" "$sol\__pycache__" /NFL /NDL /NJH /NJS /NP | Out-Null
    New-Item -ItemType Directory -Force "$e\controllers\proposed_solution\sim_logs", "$e\log" | Out-Null
}
Remove-Item Env:SAR_SNAP_ROOT -ErrorAction SilentlyContinue
Remove-Item Env:SAR_SNAP_DIR -ErrorAction SilentlyContinue
$env:SAR_OUT_DIR = "$sol\dev\eval\runs"
$env:SAR_PARALLEL = "1"
$queue = [System.Collections.Queue]::new()
Get-Content $Jobs | Where-Object { $_.Trim() -ne "" -and -not $_.StartsWith("#") } | ForEach-Object { $queue.Enqueue($_.Trim()) }
$busy = @{}
"[parN] $($queue.Count) jobs, $Slots slots, start $(Get-Date -Format HH:mm:ss)" | Out-File "$o\q_parN.log" -Append
while ($queue.Count -gt 0 -or $busy.Count -gt 0) {
    foreach ($e in @($busy.Keys)) {
        if ($busy[$e].HasExited) {
            "[parN] done $($busy[$e].Tag) $(Get-Date -Format HH:mm:ss)" | Out-File "$o\q_parN.log" -Append
            $busy.Remove($e)
        }
    }
    foreach ($e in $envs) {
        if ($queue.Count -eq 0) { break }
        if ($busy.ContainsKey($e)) { continue }
        $j = $queue.Dequeue().Split(" ")
        $exp, $ctl, $world, $seed = $j[0], $j[1], $j[2], $j[3]
        $W = $world.Substring(0, 1).ToUpper()
        $tag = "${exp}_$W$seed"
        Copy-Item "$root\_scratch\$ctl" "$e\controllers\proposed_solution\proposed_solution.py" -Force
        $a = @("-u", "-X", "utf8", "controllers\proposed_solution\dev\tools\run_batch.py",
               "--world", $world, "--runs", "1", "--tag", $tag, "--mission-s", "$MissionS",
               "--debug-vision", "--seeds", $seed)
        $p = Start-Process -PassThru -WindowStyle Hidden -FilePath $py -ArgumentList $a -WorkingDirectory $e `
            -RedirectStandardOutput "$o\q_$tag.out" -RedirectStandardError "$o\q_$tag.err"
        $p | Add-Member -NotePropertyName Tag -NotePropertyValue $tag
        $busy[$e] = $p
        "[parN] start $tag in $(Split-Path $e -Leaf) $(Get-Date -Format HH:mm:ss)" | Out-File "$o\q_parN.log" -Append
        Start-Sleep 25
    }
    Start-Sleep 10
}
Copy-Item "$root\_scratch\ps_kept.py" "$sol\proposed_solution.py" -Force
"[parN] all done $(Get-Date -Format HH:mm:ss)" | Out-File "$o\q_parN.log" -Append
"done" | Out-File "$o\q_parN.done"
