# Installe le strict minimum pour lancer les tests (pio test -e native) sur Windows, en ligne de commande
# uniquement (VS Code non requis) :
#   1. uv (script officiel Astral)
#   2. PlatformIO Core, installé comme outil uv (python géré par uv, pas de Python système requis)
#   3. un compilateur C/C++ (g++), exigé par la plateforme "native" de PlatformIO
#      (winget : WinLibs ; à défaut Chocolatey : mingw)
#
# Installation seule :
#   powershell -ExecutionPolicy Bypass -File .\setup_tests.ps1
# Installation puis exécution des tests :
#   powershell -ExecutionPolicy Bypass -File .\setup_tests.ps1 -Lancer
# Ensuite, dans un NOUVEAU terminal :  pio test -e native

param([switch]$Lancer)

$ErrorActionPreference = "Stop"

function Actualiser-Path {
    $machine = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $user = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$env:USERPROFILE\.local\bin;$machine;$user"
}

# --- 1. uv (script d'installation officiel) -------------------------------------------------------
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    Write-Host "Installation de uv..."
    Invoke-RestMethod https://astral.sh/uv/install.ps1 | Invoke-Expression
    Actualiser-Path
} else {
    Write-Host "uv déjà installé."
}

# --- 2. PlatformIO Core ---------------------------------------------------------------------------
if (-not (Get-Command pio -ErrorAction SilentlyContinue)) {
    Write-Host "Installation de PlatformIO Core via uv..."
    uv tool install platformio
    uv tool update-shell
    Actualiser-Path
} else {
    Write-Host "PlatformIO (pio) déjà disponible."
}

# --- 3. Compilateur g++ ---------------------------------------------------------------------------
function Installer-Compilateur {
    if (Get-Command winget -ErrorAction SilentlyContinue) {
        Write-Host "Installation de g++ (WinLibs) via winget..."
        winget install -e --id BrechtSanders.WinLibs.POSIX.UCRT --accept-package-agreements --accept-source-agreements
        if ($LASTEXITCODE -eq 0) { return }
        Write-Warning "L'installation via winget a échoué (code $LASTEXITCODE)."
    }
    if (Get-Command choco -ErrorAction SilentlyContinue) {
        Write-Host "Installation de g++ (mingw) via Chocolatey..."
        choco install mingw -y
        if ($LASTEXITCODE -eq 0) { return }
        Write-Warning "L'installation via Chocolatey a échoué (code $LASTEXITCODE)."
    }
    throw "Impossible d'installer g++ automatiquement : installez-le à la main et ajoutez-le au PATH."
}

if (-not (Get-Command g++ -ErrorAction SilentlyContinue)) {
    Installer-Compilateur
    Actualiser-Path
} else {
    Write-Host "g++ déjà installé."
}

if (-not (Get-Command g++ -ErrorAction SilentlyContinue)) {
    Write-Warning "g++ est installé mais pas encore visible dans ce terminal : ouvrez-en un nouveau avant de lancer les tests."
    if ($Lancer) { throw "-Lancer ignoré : relancez le script dans un nouveau terminal." }
}

Write-Host ""
if ($Lancer) {
    Push-Location $PSScriptRoot
    try { pio test -e native } finally { Pop-Location }
} else {
    Write-Host "Terminé. Ouvrez un nouveau terminal (pour recharger le PATH) puis lancez : pio test -e native"
}
