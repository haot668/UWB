# Remove FUSION_ANCHOR_RANK from Slave project — all slaves use auto-discovery
$f = "D:\Work\UWB_project\UWB_V0.0.1\uwb_anchor\DWS3000_Anchor_Slave.uvprojx"
$c = [System.IO.File]::ReadAllText($f, [System.Text.Encoding]::UTF8)

# Replace ", FUSION_ANCHOR_RANK=1" (and 2,3,4) with empty string
$c = $c -replace ', FUSION_ANCHOR_RANK=\d', ''

Copy-Item $f "$f.unified.bak" -Force
[System.IO.File]::WriteAllText($f, $c, [System.Text.Encoding]::UTF8)
Write-Host "Slave project: FUSION_ANCHOR_RANK removed from all targets"
Write-Host "All targets now produce identical binary (auto-discovery)"
