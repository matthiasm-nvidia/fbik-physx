$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
Import-Module Microsoft.PowerShell.Utility
Import-Module Microsoft.PowerShell.Archive
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
$projectRoot = Split-Path $PSScriptRoot -Parent
$manifest = Get-Content -LiteralPath (Join-Path $projectRoot 'dependencies.json') -Raw | ConvertFrom-Json
$dependencyRoot = Join-Path $projectRoot 'deps'
$downloadRoot = Join-Path $dependencyRoot 'downloads'
New-Item -ItemType Directory -Path $downloadRoot -Force | Out-Null

foreach ($package in $manifest.packages) {
    $destination = Join-Path $dependencyRoot $package.name
    $marker = Join-Path $destination '.package-sha256'
    if (Test-Path -LiteralPath $destination) {
        if ((Test-Path -LiteralPath $marker) -and
            ((Get-Content -LiteralPath $marker -Raw).Trim() -eq $package.sha256) -and
            (Test-Path -LiteralPath (Join-Path $destination $package.requiredFile))) {
            Write-Output "$($package.name) $($package.version) is installed."
            continue
        }
        throw "Unrecognized SDK directory: $destination. Move it aside before installing the pinned package."
    }

    $archive = Join-Path $downloadRoot ($package.name + '-' + $package.version + '.zip')
    if (-not (Test-Path -LiteralPath $archive)) {
        Write-Output "Downloading $($package.name) $($package.version)..."
        Invoke-WebRequest -UseBasicParsing -Uri $package.url -OutFile $archive
    }
    $actualHash = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actualHash -ne $package.sha256) {
        throw "Checksum mismatch: $archive. Remove this incomplete or incorrect archive and rerun setup_deps.bat."
    }

    # Extract to a unique sibling first; never overwrite an existing installation.
    $staging = Join-Path $dependencyRoot ($package.name + '-extract-' + [Guid]::NewGuid().ToString('N'))
    Write-Output "Extracting $($package.name)..."
    Expand-Archive -LiteralPath $archive -DestinationPath $staging
    if (-not (Test-Path -LiteralPath (Join-Path $staging $package.requiredFile))) {
        throw "Unexpected archive layout in $staging. Installation was not activated."
    }
    Set-Content -LiteralPath (Join-Path $staging '.package-sha256') -Value $package.sha256 -Encoding ASCII
    # Both absolute paths are constructed beneath the fixed project deps directory.
    $allowedPrefix = [IO.Path]::GetFullPath($dependencyRoot) + [IO.Path]::DirectorySeparatorChar
    foreach ($candidate in @($staging, $destination)) {
        if (-not [IO.Path]::GetFullPath($candidate).StartsWith($allowedPrefix, [StringComparison]::OrdinalIgnoreCase)) {
            throw 'SDK installation path escaped the project deps directory.'
        }
    }
    Move-Item -LiteralPath $staging -Destination $destination
    Write-Output "Installed $($package.name) $($package.version)."
}
