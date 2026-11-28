param([string]$World = "small", [int]$Seed = 6, [string]$Tag = "snapS6", [string]$Ctl = "ps_kept.py")
# One debug run with camera snapshots of every candidate frame (1/s), alone on sar-env.
$root = "$env:USERPROFILE\projects\sar"
$main = "$root\sar-env"; $sol = "$main\controllers\proposed_solution"
$py = "$main\.venv\Scripts\python.exe"; $o = "$main\_eval"
while (Get-CimInstance Win32_Process -Filter "Name like 'python%'" | Where-Object { $_.CommandLine -match 'run_batch' }) { Start-Sleep 15 }
Copy-Item "$root\_scratch\$Ctl" "$sol\proposed_solution.py" -Force
$env:SAR_SNAP_DIR = "$o\snaps\$Tag"
New-Item -ItemType Directory -Force $env:SAR_SNAP_DIR | Out-Null
$env:SAR_OUT_DIR = "$sol\dev\eval\runs"
Remove-Item Env:SAR_PARALLEL -ErrorAction SilentlyContinue
"[snap] start $Tag $(Get-Date -Format HH:mm:ss)" | Out-File "$o\q_parN.log" -Append
& $py -u -X utf8 "$sol\dev\tools\run_batch.py" --world $World --runs 1 --tag $Tag --mission-s 600 --debug-vision --seeds $Seed *> "$o\q_$Tag.out"
Copy-Item "$root\_scratch\ps_kept.py" "$sol\proposed_solution.py" -Force
Remove-Item Env:SAR_SNAP_DIR -ErrorAction SilentlyContinue
"[snap] done $Tag $(Get-Date -Format HH:mm:ss)" | Out-File "$o\q_parN.log" -Append
