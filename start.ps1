param([switch]$NoBrowser)
$ErrorActionPreference='Stop'
$demoRoot=Split-Path -Parent $MyInvocation.MyCommand.Path
$url='http://127.0.0.1:8878'
$healthy=$false
try { $h=Invoke-RestMethod "$url/api/health" -TimeoutSec 2; $healthy=($h.app -eq 'zhiluheng_demo' -and $h.root.Replace('/','\') -eq $demoRoot) } catch {}
if (-not $healthy) {
    $pythonPath=Join-Path $demoRoot 'runtime\python\python.exe'
    $serverPath=Join-Path $demoRoot 'server.py'
    $outPath=Join-Path $demoRoot 'outputs'
    New-Item -ItemType Directory -Path $outPath -Force | Out-Null
    $proc=Start-Process -FilePath $pythonPath -ArgumentList @('-u',('"'+$serverPath+'"'),'--port','8878') -WorkingDirectory $demoRoot -WindowStyle Hidden -RedirectStandardOutput (Join-Path $outPath 'server.stdout.log') -RedirectStandardError (Join-Path $outPath 'server.stderr.log') -PassThru
    $proc.Id | Set-Content -LiteralPath (Join-Path $outPath 'server.pid') -Encoding ascii
    for ($i=0;$i -lt 30;$i++) {
        Start-Sleep -Milliseconds 500
        try { $h=Invoke-RestMethod "$url/api/health" -TimeoutSec 1; if($h.app -eq 'zhiluheng_demo' -and $h.root.Replace('/','\') -eq $demoRoot){$healthy=$true;break} } catch {}
        if($proc.HasExited){break}
    }
    if(-not $healthy){throw 'Demo 启动失败，请查看 outputs/server.stderr.log。'}
}
Write-Output "智路衡已启动：$url"
if(-not $NoBrowser){Start-Process $url}
