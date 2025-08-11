param(
    [ValidateSet("Release", "Debug")]
    [string]$BuildType = "Release",
    
    [string]$OrbbecSDKPath = "",
    
    [string]$OpenCVDir = "",
    
    [string]$VcpkgRoot = "",
    
    [string]$Generator = "Visual Studio 16 2019",
    
    [switch]$Clean,
    
    [switch]$Help
)

function Show-Help {
    Write-Host @"
ORB-SLAM3 Windows Build Script

Usage: .\build_windows.ps1 [OPTIONS]

Options:
  -BuildType <Release|Debug>    Build configuration (default: Release)
  -OrbbecSDKPath <path>         Path to Orbbec SDK (auto-detected if not specified)
  -OpenCVDir <path>             Path to OpenCV cmake directory
  -VcpkgRoot <path>             Path to vcpkg root directory (auto-detected if not specified)
  -Generator <generator>        CMake generator (default: "Visual Studio 16 2019")
  -Clean                        Clean build directory before building
  -Help                         Show this help message

Examples:
  .\build_windows.ps1
  .\build_windows.ps1 -BuildType Debug
  .\build_windows.ps1 -OrbbecSDKPath "C:\OrbbecSDK"
  .\build_windows.ps1 -Clean -BuildType Release
  .\build_windows.ps1 -VcpkgRoot "C:\vcpkg"

Requirements:
  - CMake 3.10 or higher
  - Visual Studio 2019 or higher
  - OpenCV 4.4 or higher
  - Eigen3
  - Pangolin
  - Orbbec SDK (optional)
"@
}

if ($Help) {
    Show-Help
    exit 0
}

# Script configuration
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir = Join-Path $ScriptDir "build"
$CMakeArgs = @()

# Auto-detect vcpkg if not specified
if ([string]::IsNullOrEmpty($VcpkgRoot)) {
    $PossibleVcpkgPaths = @(
        "C:\vcpkg",
        "C:\dev\vcpkg",
        "C:\tools\vcpkg",
        "${env:VCPKG_ROOT}",
        (Get-Command vcpkg -ErrorAction SilentlyContinue | Split-Path -Parent | Split-Path -Parent)
    )
    
    foreach ($Path in $PossibleVcpkgPaths) {
        if (![string]::IsNullOrEmpty($Path) -and (Test-Path $Path) -and (Test-Path (Join-Path $Path "scripts\buildsystems\vcpkg.cmake"))) {
            $VcpkgRoot = $Path
            Write-Host "Auto-detected vcpkg at: $Path" -ForegroundColor Green
            break
        }
    }
}

# Auto-detect Orbbec SDK if not specified
if ([string]::IsNullOrEmpty($OrbbecSDKPath)) {
    $PossiblePaths = @(
        (Join-Path $ScriptDir "..\OrbbecSDK"),
        (Join-Path $ScriptDir "..\OrbbecSDK_v2.4.8_202507032159_ec8e346_win_x64"),
        "C:\OrbbecSDK",
        "C:\Program Files\OrbbecSDK"
    )
    
    foreach ($Path in $PossiblePaths) {
        if (Test-Path $Path) {
            $OrbbecSDKPath = $Path
            Write-Host "Auto-detected Orbbec SDK at: $Path" -ForegroundColor Green
            break
        }
    }
}

Write-Host "================================================================" -ForegroundColor Cyan
Write-Host "Building ORB-SLAM3 for Windows" -ForegroundColor Cyan
Write-Host "================================================================" -ForegroundColor Cyan
Write-Host "Build Type: $BuildType" -ForegroundColor Green
Write-Host "Script Directory: $ScriptDir" -ForegroundColor Green
Write-Host "Build Directory: $BuildDir" -ForegroundColor Green
Write-Host "CMake Generator: $Generator" -ForegroundColor Green

if ([string]::IsNullOrEmpty($VcpkgRoot)) {
    Write-Host "vcpkg: Not found (manual dependency management)" -ForegroundColor Yellow
} else {
    Write-Host "vcpkg Root: $VcpkgRoot" -ForegroundColor Green
}

if ([string]::IsNullOrEmpty($OrbbecSDKPath)) {
    Write-Host "Orbbec SDK: Not found (will build without Orbbec support)" -ForegroundColor Yellow
} else {
    Write-Host "Orbbec SDK Path: $OrbbecSDKPath" -ForegroundColor Green
}

Write-Host "================================================================" -ForegroundColor Cyan

# Check prerequisites
Write-Host "Checking prerequisites..." -ForegroundColor Blue

# Check CMake
try {
    $cmakeVersion = & cmake --version 2>&1 | Select-Object -First 1
    Write-Host "CMake found: $cmakeVersion" -ForegroundColor Green
} catch {
    Write-Host "CMake not found. Please install CMake and add it to PATH." -ForegroundColor Red
    exit 1
}

# Check Visual Studio
try {
    if ($Generator -like "*Visual Studio*") {
        $vsWhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
        if (Test-Path $vsWhere) {
            $vsInfo = & $vsWhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property displayName
            if ($vsInfo) {
                Write-Host "Visual Studio found: $vsInfo" -ForegroundColor Green
            } else {
                Write-Host "Visual Studio with C++ tools not found." -ForegroundColor Red
                exit 1
            }
        }
    }
} catch {
    Write-Host "Could not verify Visual Studio installation" -ForegroundColor Yellow
}

# Check vcpkg
if (![string]::IsNullOrEmpty($VcpkgRoot)) {
    $vcpkgToolchain = Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake"
    if (Test-Path $vcpkgToolchain) {
        Write-Host "vcpkg toolchain found and validated" -ForegroundColor Green
    } else {
        Write-Host "vcpkg root exists but toolchain not found: $vcpkgToolchain" -ForegroundColor Yellow
        Write-Host "  Building without vcpkg integration..." -ForegroundColor Yellow
        $VcpkgRoot = ""
    }
}

# Check Orbbec SDK
if (![string]::IsNullOrEmpty($OrbbecSDKPath)) {
    if (Test-Path $OrbbecSDKPath) {
        $includeDir = Join-Path $OrbbecSDKPath "include\libobsensor"
        $libDir = Join-Path $OrbbecSDKPath "lib"
        
        if ((Test-Path $includeDir) -and (Test-Path $libDir)) {
            Write-Host "Orbbec SDK found and validated" -ForegroundColor Green
        } else {
            Write-Host "Orbbec SDK path exists but structure is invalid" -ForegroundColor Yellow
            Write-Host "  Expected: $includeDir" -ForegroundColor Yellow
            Write-Host "  Expected: $libDir" -ForegroundColor Yellow
        }
    } else {
        Write-Host "Orbbec SDK path does not exist: $OrbbecSDKPath" -ForegroundColor Red
        Write-Host "  Building without Orbbec support..." -ForegroundColor Yellow
        $OrbbecSDKPath = ""
    }
}

# Clean build directory if requested
if ($Clean) {
    if (Test-Path $BuildDir) {
        Write-Host "Cleaning build directory..." -ForegroundColor Blue
        Remove-Item -Recurse -Force $BuildDir -ErrorAction SilentlyContinue
        Start-Sleep -Seconds 1  # Give time for file system to catch up
        Write-Host "Build directory cleaned" -ForegroundColor Green
    }
}

# Create build directory
if (!(Test-Path $BuildDir)) {
    Write-Host "Creating build directory: $BuildDir" -ForegroundColor Blue
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

# Prepare CMake arguments
$CMakeArgs = @(
    "-G", $Generator,
    "-A", "x64",
    "-DCMAKE_BUILD_TYPE=$BuildType"
)

# Add vcpkg toolchain if available
if (![string]::IsNullOrEmpty($VcpkgRoot)) {
    $vcpkgToolchain = Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake"
    $CMakeArgs += "-DCMAKE_TOOLCHAIN_FILE=$vcpkgToolchain"
    $CMakeArgs += "-DVCPKG_TARGET_TRIPLET=x64-windows"
}

# Add OpenCV directory if specified
if (![string]::IsNullOrEmpty($OpenCVDir)) {
    $CMakeArgs += "-DOpenCV_DIR=$OpenCVDir"
}

# Add Orbbec SDK configuration
if (![string]::IsNullOrEmpty($OrbbecSDKPath)) {
    $CMakeArgs += @(
        "-DBUILD_WITH_ORBBEC=ON",
        "-DORBBEC_SDK_ROOT=$OrbbecSDKPath"
    )
} else {
    $CMakeArgs += "-DBUILD_WITH_ORBBEC=OFF"
}

# Add source directory
$CMakeArgs += $ScriptDir

Write-Host ""
Write-Host "================================================================" -ForegroundColor Cyan
Write-Host "Configuring CMake..." -ForegroundColor Cyan
Write-Host "================================================================" -ForegroundColor Cyan

# Change to build directory
Push-Location $BuildDir

try {
    Write-Host "Running: cmake $($CMakeArgs -join ' ')" -ForegroundColor Blue
    
    # Run CMake configuration
    $configResult = & cmake @CMakeArgs 2>&1
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host ""
        Write-Host "CMake configuration failed!" -ForegroundColor Red
        Write-Host ""
        Write-Host "Error output:" -ForegroundColor Red
        $configResult | ForEach-Object { Write-Host $_ -ForegroundColor Red }
        Write-Host ""
        exit 1
    }
    
    Write-Host "CMake configuration successful" -ForegroundColor Green

    Write-Host ""
    Write-Host "================================================================" -ForegroundColor Cyan
    Write-Host "Building project..." -ForegroundColor Cyan
    Write-Host "================================================================" -ForegroundColor Cyan

    # Build the project
    Write-Host "Running: cmake --build . --config $BuildType --parallel" -ForegroundColor Blue
    
    $buildResult = & cmake --build . --config $BuildType --parallel $env:NUMBER_OF_PROCESSORS 2>&1
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host ""
        Write-Host "Build failed!" -ForegroundColor Red
        Write-Host ""
        Write-Host "Error output:" -ForegroundColor Red
        $buildResult | Where-Object { $_ -match "error" } | ForEach-Object { Write-Host $_ -ForegroundColor Red }
        exit 1
    }
    
    Write-Host "Build completed successfully!" -ForegroundColor Green

} finally {
    Pop-Location
}

Write-Host ""
Write-Host "================================================================" -ForegroundColor Cyan
Write-Host "Post-build tasks..." -ForegroundColor Cyan
Write-Host "================================================================" -ForegroundColor Cyan

# Copy necessary DLLs if Orbbec SDK is used
if (![string]::IsNullOrEmpty($OrbbecSDKPath)) {
    Write-Host "Copying Orbbec SDK DLLs..." -ForegroundColor Blue
    
    $libDir = Join-Path $OrbbecSDKPath "lib"
    $rgbdDir = Join-Path $ScriptDir "Examples\RGB-D"
    $rgbdInertialDir = Join-Path $ScriptDir "Examples\RGB-D-Inertial"
    
    # Copy DLLs to RGB-D examples directory
    if (Test-Path $libDir) {
        $dlls = Get-ChildItem -Path $libDir -Filter "*.dll"
        foreach ($dll in $dlls) {
            Copy-Item $dll.FullName $rgbdDir -Force -ErrorAction SilentlyContinue
            Copy-Item $dll.FullName $rgbdInertialDir -Force -ErrorAction SilentlyContinue
        }
        
        # Copy extensions folder if it exists
        $extensionsDir = Join-Path $libDir "extensions"
        if (Test-Path $extensionsDir) {
            $rgbdExtDir = Join-Path $rgbdDir "extensions"
            $rgbdInertialExtDir = Join-Path $rgbdInertialDir "extensions"
            
            if (!(Test-Path $rgbdExtDir)) { New-Item -ItemType Directory -Path $rgbdExtDir | Out-Null }
            if (!(Test-Path $rgbdInertialExtDir)) { New-Item -ItemType Directory -Path $rgbdInertialExtDir | Out-Null }
            
            Copy-Item -Path "$extensionsDir\*" -Destination $rgbdExtDir -Recurse -Force -ErrorAction SilentlyContinue
            Copy-Item -Path "$extensionsDir\*" -Destination $rgbdInertialExtDir -Recurse -Force -ErrorAction SilentlyContinue
        }
        
        Write-Host "Orbbec SDK DLLs copied to examples directories" -ForegroundColor Green
    }
}

Write-Host ""
Write-Host "================================================================" -ForegroundColor Cyan
Write-Host "Build Summary" -ForegroundColor Cyan
Write-Host "================================================================" -ForegroundColor Cyan

Write-Host "Build Type: $BuildType" -ForegroundColor Green
Write-Host "Build Directory: $BuildDir" -ForegroundColor Green
Write-Host "Library Output: $(Join-Path $ScriptDir 'lib')" -ForegroundColor Green
Write-Host "Examples Output: $(Join-Path $ScriptDir 'Examples')" -ForegroundColor Green

if (![string]::IsNullOrEmpty($OrbbecSDKPath)) {
    Write-Host ""
    Write-Host "ORB-SLAM3 built with Orbbec SDK support!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Available Orbbec examples:" -ForegroundColor Blue
    Write-Host "    $(Join-Path $ScriptDir 'Examples\RGB-D\rgbd_orbbec_gemini335.exe')" -ForegroundColor Blue
    Write-Host "    $(Join-Path $ScriptDir 'Examples\RGB-D-Inertial\rgbd_inertial_orbbec_gemini335.exe')" -ForegroundColor Blue
} else {
    Write-Host ""
    Write-Host "ORB-SLAM3 built without Orbbec SDK support" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "================================================================" -ForegroundColor Cyan
Write-Host "Next Steps" -ForegroundColor Cyan
Write-Host "================================================================" -ForegroundColor Cyan

Write-Host ""
Write-Host "Example usage:" -ForegroundColor Green
Write-Host "cd Examples\RGB-D" -ForegroundColor Green
Write-Host ".\rgbd_orbbec_gemini335.exe ..\..\Vocabulary\ORBvoc.txt camera.yaml" -ForegroundColor Green

Write-Host ""
Write-Host "Debugging tips:" -ForegroundColor Yellow
Write-Host "  If build fails, run this for cmake:" -ForegroundColor Yellow
Write-Host "  cmake -G "Visual Studio 16 2019" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -DBUILD_WITH_ORBBEC=ON -DORBBEC_SDK_ROOT=C:\Users\31110\Downloads\work\O-depth-Cam\OrbbecSDK .." -ForegroundColor Blue
Write-Host "  To build" -ForegroundColor Yellow
Write-Host "  cmake --build . --config Release --parallel $env:NUMBER_OF_PROCESSORS" -ForegroundColor Blue

Write-Host ""
Write-Host "Build completed successfully!" -ForegroundColor Green