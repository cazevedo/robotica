<#
    lab.ps1 - Windows-side driver for the Lab 0 container.
    Robotics (02000537) 2026/2027 - UFACTORY Lite 6 / ROS 2 Jazzy.

    Everything is just `docker compose` underneath; each command prints the
    line it runs so you can use docker directly whenever you prefer.

        .\lab.ps1 build     build the image (once, 45-90 min)
        .\lab.ps1 up        start the container
        .\lab.ps1 shell     open a terminal inside it
        .\lab.ps1 help      the rest
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Command = 'help',

    [Parameter(Position = 1, ValueFromRemainingArguments = $true)]
    [string[]]$Rest
)

# Native tools report failure through $LASTEXITCODE, not PowerShell errors;
# 'Stop' would turn harmless stderr chatter from docker into a script abort.
$ErrorActionPreference = 'Continue'
$Root    = $PSScriptRoot
$Service = 'lab'

function Write-Head($msg) { Write-Host "==> $msg" -ForegroundColor Cyan }
function Write-Warn($msg) { Write-Host "!!  $msg" -ForegroundColor Yellow }
function Write-Bad ($msg) { Write-Host "!!  $msg" -ForegroundColor Red }

function Invoke-Compose {
    param([string[]]$ComposeArgs)
    Write-Host "`$ docker compose $($ComposeArgs -join ' ')" -ForegroundColor DarkGray
    & docker compose @ComposeArgs
    # Deliberately returns nothing. A native command's stdout is part of the
    # enclosing function's output, so `return $LASTEXITCODE` would hand the
    # caller the entire build log with the exit code tacked on the end - and
    # `if ($code -eq 0)` on that array is False even on a successful build.
    # Callers read $LASTEXITCODE directly; it survives the function call.
}

$DockerDesktopExe = Join-Path $env:ProgramFiles 'Docker\Docker\Docker Desktop.exe'
$DockerBinDir     = Join-Path $env:ProgramFiles 'Docker\Docker\resources\bin'

function Assert-Docker {
    # A terminal opened BEFORE Docker Desktop was installed still carries the
    # old PATH. Re-read it from the environment rather than telling you to
    # close the window.
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                    [Environment]::GetEnvironmentVariable('Path', 'User')
    }
    # Still nothing, but the binary is where the installer puts it: use it.
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        if (Test-Path (Join-Path $DockerBinDir 'docker.exe')) {
            $env:Path = "$DockerBinDir;$env:Path"
            Write-Warn "docker was not on this terminal's PATH - using $DockerBinDir."
            Write-Warn "A terminal opened from now on will find it by itself."
        }
    }
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        Write-Bad "Docker is not installed (or not on PATH)."
        Write-Host ""
        Write-Host "  Install Docker Desktop, then reopen this terminal:"
        Write-Host "      winget install -e --id Docker.DockerDesktop" -ForegroundColor White
        Write-Host ""
        Write-Host "  Start it once, and in Settings make sure"
        Write-Host "  'Use the WSL 2 based engine' is ticked."
        exit 1
    }

    & docker info --format '{{.ServerVersion}}' | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Bad "The docker command works, but the engine is not responding."
        Write-Host ""
        if (-not (Get-Process 'Docker Desktop' -ErrorAction SilentlyContinue)) {
            Write-Host "  Docker Desktop is not running. Start it:"
            if (Test-Path $DockerDesktopExe) {
                Write-Host "      Start-Process `"$DockerDesktopExe`"" -ForegroundColor White
            } else {
                Write-Host "      (from the Start menu)" -ForegroundColor White
            }
            Write-Host ""
            Write-Host "  First launch asks you to accept the terms; you do NOT need"
            Write-Host "  a Docker account. Wait until the whale icon stops animating,"
            Write-Host "  then run this command again."
        } else {
            Write-Host "  Docker Desktop is running but still starting up. Wait for the"
            Write-Host "  whale icon to stop animating, then run this command again."
        }
        exit 1
    }
}

function Assert-Env {
    $envFile = Join-Path $Root '.env'
    if (-not (Test-Path $envFile)) {
        Write-Warn ".env was missing - created it from .env.example."
        Write-Warn "Open it and set ROS_DOMAIN_ID to your assigned number."
        Copy-Item (Join-Path $Root '.env.example') $envFile
    }
}

function Get-EnvValue([string]$Key) {
    $envFile = Join-Path $Root '.env'
    if (-not (Test-Path $envFile)) { return $null }
    foreach ($line in Get-Content $envFile) {
        if ($line -match "^\s*$([regex]::Escape($Key))\s*=\s*(.*)$") {
            return $Matches[1].Trim()
        }
    }
    return $null
}

function Set-EnvValue([string]$Key, [string]$Value) {
    $envFile = Join-Path $Root '.env'
    $lines   = Get-Content $envFile
    $found   = $false
    $out     = foreach ($line in $lines) {
        if ($line -match "^\s*$([regex]::Escape($Key))\s*=") {
            $found = $true
            "$Key=$Value"
        } else {
            $line
        }
    }
    if (-not $found) { $out = $out + "$Key=$Value" }
    Set-Content -Path $envFile -Value $out -Encoding utf8
}

function Test-Running {
    $id = & docker compose ps -q $Service
    if ([string]::IsNullOrWhiteSpace($id)) { return $false }
    $state = & docker inspect -f '{{.State.Running}}' $id
    return ($state -eq 'true')
}

function Ensure-Up {
    if (-not (Test-Running)) {
        Write-Head "container is not running - starting it"
        Invoke-Compose @('up', '-d') | Out-Null
        Start-Sleep -Milliseconds 800
    }
}

function Show-Help {
@"
lab.ps1 - Lab 0 container (UFACTORY Lite 6 / ROS 2 Jazzy / Gazebo Harmonic)

  SETUP  (once)
    .\lab.ps1 build            build the image. 45-90 min, mostly downloads.
    .\lab.ps1 up               start the container in the background
    .\lab.ps1 doctor           check everything is in place

  EVERY DAY
    .\lab.ps1 shell            open a terminal inside the container.
                               Run it again for each "new terminal" the
                               handout asks for. Type 'lab' in there.
    .\lab.ps1 run <cmd...>     run one command inside, e.g.
                                 .\lab.ps1 run lab test 2
    .\lab.ps1 down             stop the container (your files are kept)
    .\lab.ps1 restart          stop, start, re-read .env
    .\lab.ps1 status           what is running
    .\lab.ps1 logs             container output
    .\lab.ps1 vnc              open the browser display (DISPLAY_MODE=vnc)
    .\lab.ps1 code             open this folder in VS Code (Dev Containers)

  THE GAZEBO SWITCH
    .\lab.ps1 gazebo off       block Gazebo: no physics engine, no CPU load.
                               RViz, MoveIt and the real robot still work.
    .\lab.ps1 gazebo on        allow it again (Tests 6 and 7 need it)
    .\lab.ps1 gazebo           show the current setting
                               Takes effect immediately, no restart.

  MAINTENANCE
    .\lab.ps1 rebuild          rebuild the image, reusing cached layers
    .\lab.ps1 rebuild-xarm     rebuild with a fresh clone of xarm_ros2
    .\lab.ps1 nuke             delete container AND volumes.
                               .\src and .\shared on Windows are NOT touched.

  WHERE YOUR WORK LIVES
    .\src      -> ~/dev_ws/src   your ROS packages. Edit in Windows, build
                                 inside. git init this folder and commit.
    .\shared   -> ~/shared       files in and out (the view_frames PDF, notes)
    Anything else inside the container is disposable.
"@ | Write-Host
}

# --------------------------------------------------------------------------- #
Push-Location $Root
try {
    switch ($Command.ToLower()) {

        { $_ -in 'help', '-h', '--help', '/?' } { Show-Help; break }

        'build' {
            Assert-Docker; Assert-Env
            Write-Head "building robotics-lab0:jazzy - this takes 45-90 minutes"
            Write-Host  "    Downloads ~4 GB and compiles xarm_ros2. Go do something else."
            Write-Host  ""
            $a = @('--progress', 'plain', 'build')
            if ($Rest -contains '-NoCache' -or $Rest -contains '--no-cache') { $a += '--no-cache' }
            Invoke-Compose $a
            $code = $LASTEXITCODE
            if ($code -eq 0) {
                Write-Head "image built. Next:  .\lab.ps1 up   then   .\lab.ps1 shell"
            } else {
                Write-Bad "build failed. The FIRST error in the output above is the one that matters."
            }
            exit $code
        }

        'rebuild' {
            Assert-Docker; Assert-Env
            Invoke-Compose @('--progress', 'plain', 'build')
            exit $LASTEXITCODE
        }

        'rebuild-xarm' {
            Assert-Docker; Assert-Env
            $stamp = Get-Date -Format 'yyyy-MM-dd-HHmmss'
            Write-Head "rebuilding with a fresh clone of xarm_ros2 ($stamp)"
            Invoke-Compose @('--progress', 'plain', 'build', '--build-arg', "XARM_CACHEBUST=$stamp")
            exit $LASTEXITCODE
        }

        { $_ -in 'up', 'start' } {
            Assert-Docker; Assert-Env
            Invoke-Compose @('up', '-d')
            $code = $LASTEXITCODE
            if ($code -eq 0) {
                Write-Head "running. Open a terminal with:  .\lab.ps1 shell"
                $mode = Get-EnvValue 'DISPLAY_MODE'
                if ($mode -eq 'vnc') { Write-Head "browser display: http://localhost:6080/" }
            }
            exit $code
        }

        { $_ -in 'down', 'stop' } {
            Assert-Docker
            Write-Head "stopping. Your code in .\src and the build volume are kept."
            Invoke-Compose @('down')
            exit $LASTEXITCODE
        }

        'restart' {
            Assert-Docker; Assert-Env
            Invoke-Compose @('down')
            Invoke-Compose @('up', '-d')
            exit $LASTEXITCODE
        }

        { $_ -in 'shell', 'sh', 'bash', 'term', 'terminal' } {
            Assert-Docker; Assert-Env; Ensure-Up
            & docker compose exec $Service bash
            exit $LASTEXITCODE
        }

        { $_ -in 'run', 'exec' } {
            Assert-Docker; Assert-Env; Ensure-Up
            if (-not $Rest -or $Rest.Count -eq 0) {
                Write-Bad "usage: .\lab.ps1 run <command...>   e.g.  .\lab.ps1 run lab doctor"
                exit 1
            }
            & docker compose exec $Service @Rest
            exit $LASTEXITCODE
        }

        'gazebo' {
            Assert-Docker; Assert-Env
            $arg = ''
            if ($Rest -and $Rest.Count -gt 0) { $arg = $Rest[0].ToLower() }
            switch ($arg) {
                { $_ -in 'on', '1', 'enable' } {
                    Set-EnvValue 'LAB_GAZEBO' '1'
                    if (Test-Running) { & docker compose exec $Service lab gazebo on }
                    else { Write-Head "Gazebo enabled in .env (container is not running)." }
                    break
                }
                { $_ -in 'off', '0', 'disable' } {
                    Set-EnvValue 'LAB_GAZEBO' '0'
                    if (Test-Running) { & docker compose exec $Service lab gazebo off }
                    else { Write-Head "Gazebo disabled in .env (container is not running)." }
                    break
                }
                default {
                    Write-Host ".env:  LAB_GAZEBO=$(Get-EnvValue 'LAB_GAZEBO')"
                    if (Test-Running) { & docker compose exec $Service lab gazebo status }
                    break
                }
            }
            exit 0
        }

        { $_ -in 'doctor', 'check' } {
            Assert-Docker; Assert-Env; Ensure-Up
            & docker compose exec $Service lab doctor
            exit $LASTEXITCODE
        }

        { $_ -in 'status', 'ps' } {
            Assert-Docker
            Invoke-Compose @('ps')
            Write-Host ""
            Write-Host "Volumes:" -ForegroundColor Cyan
            & docker volume ls --filter "name=robotics-lab0"
            exit 0
        }

        'logs' {
            Assert-Docker
            Invoke-Compose (@('logs', '--tail', '200') + $Rest)
            exit $LASTEXITCODE
        }

        'vnc' {
            Ensure-Up
            Write-Head "opening http://localhost:6080/"
            Write-Host  "    (only serves anything when DISPLAY_MODE=vnc in .env)"
            Start-Process 'http://localhost:6080/'
            exit 0
        }

        'code' {
            $vscode = Get-Command code -ErrorAction SilentlyContinue
            if (-not $vscode) {
                Write-Bad "VS Code's 'code' command is not on PATH."
                Write-Host "  Install VS Code + the 'Dev Containers' extension, then rerun."
                exit 1
            }
            Write-Head "opening VS Code. Use: Reopen in Container."
            & code $Root
            exit 0
        }

        'nuke' {
            Assert-Docker
            Write-Warn "This deletes the container and its volumes:"
            Write-Warn "  - everything built inside ~/dev_ws (build/install/log)"
            Write-Warn "  - the Gazebo model cache and ROS logs"
            Write-Warn "Your .\src and .\shared folders on Windows are NOT touched."
            $ans = Read-Host "Type 'yes' to continue"
            if ($ans -ne 'yes') { Write-Host "cancelled."; exit 0 }
            Invoke-Compose @('down', '-v')
            exit $LASTEXITCODE
        }

        default {
            Write-Bad "unknown command: $Command"
            Write-Host ""
            Show-Help
            exit 1
        }
    }
}
finally {
    Pop-Location
}
