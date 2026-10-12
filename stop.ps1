$ErrorActionPreference='Stop'
$demoRoot=Split-Path -Parent $MyInvocation.MyCommand.Path
$url='http://127.0.0.1:8878'
try {
    $health=Invoke-RestMethod "$url/api/health" -TimeoutSec 2
    if($health.app -eq 'zhiluheng_demo' -and $health.root.Replace('/','\') -eq $demoRoot){
        Invoke-RestMethod "$url/api/shutdown" -Method Post -ContentType 'application/json' -Body '{"app":"zhiluheng_demo"}' | Out-Null
        Write-Output '智路衡服务和所属仿真已关闭。'
    }else{Write-Output '当前端口属于其他项目，没有停止任何程序。'}
}catch{Write-Output '未发现当前目录对应的运行服务，或关闭未成功，请检查 outputs/server.stderr.log。'}
