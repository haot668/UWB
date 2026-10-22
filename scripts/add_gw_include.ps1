$f = "D:\Work\UWB_project\UWB_V0.0.1\uwb_gateway\DWS3000_Gateway.uvprojx"
$c = [System.IO.File]::ReadAllText($f, [System.Text.Encoding]::UTF8)
$c = $c -replace '(<IncludePath>[^<]+)', '$1;..\shared\gw_ota'
Copy-Item $f "$f.bak" -Force
[System.IO.File]::WriteAllText($f, $c, [System.Text.Encoding]::UTF8)
Write-Host "Gateway include paths updated"
