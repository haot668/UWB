# add_ota_source.ps1 — Add gw_ota.c + include path to Keil projects
param([switch]$DryRun)

$projects = @(
    @{Path="D:\Work\UWB_project\UWB_V0.0.1\uwb_anchor\DWS3000_Anchor_Master.uvprojx"; Name="Master"},
    @{Path="D:\Work\UWB_project\UWB_V0.0.1\uwb_anchor\DWS3000_Anchor_Slave.uvprojx";  Name="Slave"},
    @{Path="D:\Work\UWB_project\UWB_V0.0.1\uwb_tag\DWS3000_Tag.uvprojx";              Name="Tag"}
)

$newFileXml = @"
            <File>
              <FileName>gw_ota.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\shared\gw_ota\gw_ota.c</FilePath>
            </File>
"@

$includeAdd = ";..\shared\gw_ota"

foreach ($proj in $projects) {
    Write-Host "=== $($proj.Name) ($($proj.Path)) ===" -ForegroundColor Cyan

    if (-not (Test-Path $proj.Path)) {
        Write-Host "  SKIP: file not found" -ForegroundColor Yellow
        continue
    }

    $content = Get-Content $proj.Path -Raw -Encoding UTF8

    # 1. Add file entry after fusion_anchor.c in Files section
    #    Pattern: fusion_anchor.c</FilePath>\r\n            </File>
    $marker = "fusion_anchor.c</FilePath>"
    $insertAfter = "            </File>"

    $idx = $content.IndexOf($marker)
    if ($idx -lt 0) {
        Write-Host "  WARN: fusion_anchor.c not found in project" -ForegroundColor Yellow
        continue
    }

    # Find the </File> that closes fusion_anchor.c
    $closeIdx = $content.IndexOf($insertAfter, $idx)
    if ($closeIdx -lt 0) {
        Write-Host "  WARN: closing </File> not found" -ForegroundColor Yellow
        continue
    }
    $closeIdx += $insertAfter.Length

    # Insert new file entry — but only at the FIRST occurrence (file list section)
    # Actually, we need to handle both the file list AND the project tree
    # The .uvprojx has both a <Files> section per <Group> in each <Target>
    # AND a global <Groups> section at the end.
    # We need to add to ALL of them.

    # Strategy: find ALL occurrences of "fusion_anchor.c</FilePath>" and add after each one
    $done = 0
    $newContent = ""
    $searchStart = 0
    while ($true) {
        $idx = $content.IndexOf($marker, $searchStart)
        if ($idx -lt 0) { break }

        $closeIdx = $content.IndexOf($insertAfter, $idx)
        if ($closeIdx -lt 0) { break }
        $closeIdx += $insertAfter.Length

        # Check if gw_ota.c is already after this occurrence
        $checkRegion = $content.Substring($closeIdx, [Math]::Min(200, $content.Length - $closeIdx))
        if ($checkRegion -match "gw_ota\.c") {
            Write-Host "  gw_ota.c already present at occurrence $($done+1), skip" -ForegroundColor Gray
            $newContent += $content.Substring($searchStart, $closeIdx - $searchStart)
            $searchStart = $closeIdx
            continue
        }

        # Add content before this point + insert new file
        $newContent += $content.Substring($searchStart, $closeIdx - $searchStart)
        $newContent += "`r`n$newFileXml"
        $searchStart = $closeIdx
        $done++
    }
    $newContent += $content.Substring($searchStart)

    Write-Host "  File entries added: $done"

    # 2. Add include path ..\shared\gw_ota to all <IncludePath> entries
    $newContent = $newContent -replace '(<IncludePath>[^<]+)', "`$1$includeAdd"
    Write-Host "  Include paths updated"

    if ($DryRun) {
        Write-Host "  DRY RUN — not writing" -ForegroundColor Yellow
    } else {
        # Backup original
        $backup = $proj.Path + ".bak"
        Copy-Item $proj.Path $backup -Force
        [System.IO.File]::WriteAllText($proj.Path, $newContent, [System.Text.Encoding]::UTF8)
        Write-Host "  WRITTEN (backup: $backup)" -ForegroundColor Green
    }
}

Write-Host ""
Write-Host "Done. Re-open projects in Keil and rebuild." -ForegroundColor Green
