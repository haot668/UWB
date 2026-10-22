# Add gw_ota.c to Tag Keil project only
$projPath = "D:\Work\UWB_project\UWB_V0.0.1\uwb_tag\DWS3000_Tag.uvprojx"

$newFileXml = @"
            <File>
              <FileName>gw_ota.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\shared\gw_ota\gw_ota.c</FilePath>
            </File>
"@

$content = Get-Content $projPath -Raw -Encoding UTF8
$marker = "fusion_tag.c</FilePath>"
$insertAfter = "            </File>"
$done = 0
$newContent = ""
$searchStart = 0

while ($true) {
    $idx = $content.IndexOf($marker, $searchStart)
    if ($idx -lt 0) { break }

    $closeIdx = $content.IndexOf($insertAfter, $idx)
    if ($closeIdx -lt 0) { break }
    $closeIdx += $insertAfter.Length

    $checkRegion = $content.Substring($closeIdx, [Math]::Min(200, $content.Length - $closeIdx))
    if ($checkRegion -match "gw_ota\.c") {
        Write-Host "  gw_ota.c already present, skip"
        $newContent += $content.Substring($searchStart, $closeIdx - $searchStart)
        $searchStart = $closeIdx
        continue
    }

    $newContent += $content.Substring($searchStart, $closeIdx - $searchStart)
    $newContent += "`r`n$newFileXml"
    $searchStart = $closeIdx
    $done++
}
$newContent += $content.Substring($searchStart)

# Add include path
$newContent = $newContent -replace '(<IncludePath>[^<]+)', "`$1;..\shared\gw_ota"

Write-Host "Tag: $done file entries added, include paths updated"

Copy-Item $projPath "$projPath.bak" -Force
[System.IO.File]::WriteAllText($projPath, $newContent, [System.Text.Encoding]::UTF8)
Write-Host "Written (backup: $projPath.bak)"
