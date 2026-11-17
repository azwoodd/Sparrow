# Starts a long experiment detached, logging to results\logs\<name>.log   usage: Start-Job.ps1 <name> <script.py> [args...]
param([string]$Name, [string]$Script, [Parameter(ValueFromRemainingArguments = $true)]$Rest)
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$logs = Join-Path $here "..\results\logs"; New-Item -ItemType Directory -Force $logs | Out-Null
$py = "D:\Downloads\project-sparrow\.venv\Scripts\python.exe"
$argList = @("-u", "-B", "`"$(Join-Path $here "..\src\$Script")`"") + $Rest
$p = Start-Process -FilePath $py -ArgumentList $argList -RedirectStandardOutput (Join-Path $logs "$Name.log") -RedirectStandardError (Join-Path $logs "$Name.err") -PassThru -WindowStyle Hidden
"started $Name pid $($p.Id)"
