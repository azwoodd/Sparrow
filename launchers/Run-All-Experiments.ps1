# Re-runs every experiment of this phase from the isolated copies and re-verifies that no original file changed.
#   & "D:\Downloads\Sparrow_Locomotion_Engineering_V1\launchers\Run-All-Experiments.ps1"      (about 25-40 minutes)
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$src = Join-Path $here "..\src"
$py = "D:\Downloads\project-sparrow\.venv\Scripts\python.exe"
if (-not (Test-Path $py)) { Write-Error "Python environment not found at $py"; exit 1 }
function Run($variant, $script) { $env:SPARROW_VARIANT = $variant; Write-Host "`n##### [$variant] $script $args" -ForegroundColor Cyan; & $py -u -B (Join-Path $src $script) @args }
Run v7e_baseline p0_audit.py
Run v7e_baseline p0_controls.py
Run v7e_baseline p1_authority.py
Run v7e_baseline p1_one_step.py
foreach ($v in "v7e_baseline", "v7e_stiffcontact", "v7e_stiffcontact_elliptic") {
    Run $v p1_contact.py
    foreach ($mode in "nominal", "sweep", "robust", "holdout") { Run $v p2_walk.py $mode }
}
$env:SPARROW_VARIANT = $null
Write-Host "`n##### verifying original files are unchanged" -ForegroundColor Cyan
& $py -u -B (Join-Path $src "setup_workspace.py") verify
