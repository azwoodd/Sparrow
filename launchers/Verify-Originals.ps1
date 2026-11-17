# Re-hashes every original Sparrow file recorded in manifests\originals_sha256.json and reports changed / missing / added.
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
& "D:\Downloads\project-sparrow\.venv\Scripts\python.exe" -u -B (Join-Path $here "..\src\setup_workspace.py") verify
