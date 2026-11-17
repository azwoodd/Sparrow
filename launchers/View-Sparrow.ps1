# Live MuJoCo viewer for Sparrow Locomotion Engineering V1 (real simulated physics, nothing keyframed).
#   & "D:\Downloads\Sparrow_Locomotion_Engineering_V1\launchers\View-Sparrow.ps1"                 # 10-step walk
#   & "...\View-Sparrow.ps1" one_step            # other demos:  & "...\View-Sparrow.ps1" --list
#   & "...\View-Sparrow.ps1" walk --variant v7e_baseline --friction 1.3     # original soft contact, shows the tip-over
# Keys in the 3D window: SPACE pause, R restart, N/P next/previous demo, -/= slower/faster, F follow, C collision shapes.
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$py = "D:\Downloads\project-sparrow\.venv\Scripts\python.exe"
if (-not (Test-Path $py)) { Write-Error "Python environment not found at $py"; exit 1 }
& $py -u -B (Join-Path $here "..\src\view.py") @args
