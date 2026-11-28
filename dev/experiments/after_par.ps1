param([string]$Jobs)
# Wait for the running parN batch to write q_parN.done, then start the next.
$o = "$env:USERPROFILE\projects\sar\sar-env\_eval"
$t0 = Get-Date
while (-not ((Test-Path "$o\q_parN.done") -and ((Get-Item "$o\q_parN.done").LastWriteTime -gt $t0))) { Start-Sleep 30 }
Start-Sleep 20
& "$env:USERPROFILE\projects\sar\_scratch\parN.ps1" -Jobs $Jobs
