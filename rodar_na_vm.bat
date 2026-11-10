@echo off
chcp 65001 >nul
reg add HKCU\Console /v QuickEdit /t REG_DWORD /d 0 /f >nul 2>&1

set "PASTA_SCRIPT=%~dp0"
if "%PASTA_SCRIPT:~-1%"=="\" set "PASTA_SCRIPT=%PASTA_SCRIPT:~0,-1%"
pushd "%PASTA_SCRIPT%" 2>nul

set "RAIZ_VM=%CD%"
if "%RAIZ_VM:~-1%"=="\" set "RAIZ_VM=%RAIZ_VM:~0,-1%"

echo [1/4] Encerrando processos Python antigos na VM...
taskkill /F /IM python.exe /T >nul 2>&1

echo [2/4] Desativando suspensao automatica do Windows...
powercfg /change standby-timeout-ac 0 >nul 2>&1
powercfg /change standby-timeout-dc 0 >nul 2>&1
powercfg /change hibernate-timeout-ac 0 >nul 2>&1
powercfg /change monitor-timeout-ac 0 >nul 2>&1

if exist "%RAIZ_VM%\.git" (
    echo [2.5/4] Atualizando repositorio via Git...
    git -C "%RAIZ_VM%" pull --no-rebase 2>nul
)

set "LOCAL_CACHE=%LOCALAPPDATA%\dissertacao_vm_cache"
if not exist "%LOCAL_CACHE%" mkdir "%LOCAL_CACHE%" >nul 2>&1
set "PYTHONPYCACHEPREFIX=%LOCAL_CACHE%"
set "NUMBA_CACHE_DIR=%LOCAL_CACHE%"
set "PYTHONUNBUFFERED=1"

cd /d "%RAIZ_VM%\IA" 2>nul

echo [%DATE% %TIME%] Iniciando Agente VM em %COMPUTERNAME% (Usuario: %USERNAME%) > "%RAIZ_VM%\IA\vm_cmd_debug.txt"
echo Pasta mapeada: %RAIZ_VM% (CWD: %CD% ^| Origem: %PASTA_SCRIPT%) >> "%RAIZ_VM%\IA\vm_cmd_debug.txt"

REM 1. Verifica se existe o venv local do workspace (no PC local ou VM)
if exist "%RAIZ_VM%\.venv\Scripts\python.exe" (
    echo [3/4] Iniciando Agente com Python VENV do Workspace: "%RAIZ_VM%\.venv\Scripts\python.exe"
    echo [3/4] Iniciando Agente com Python VENV do Workspace: "%RAIZ_VM%\.venv\Scripts\python.exe" >> "%RAIZ_VM%\IA\vm_cmd_debug.txt"
    "%RAIZ_VM%\.venv\Scripts\python.exe" -u "%RAIZ_VM%\IA\agente_vm.py"
    goto :fim
)

set "PY_VENV=C:\Users\a69613\PycharmProjects\Dissertacao\.venv\Scripts\python.exe"
if exist "%PY_VENV%" (
    echo [3/4] Iniciando Agente com Python VENV: "%PY_VENV%"
    echo [3/4] Iniciando Agente com Python VENV: "%PY_VENV%" >> "%RAIZ_VM%\IA\vm_cmd_debug.txt"
    "%PY_VENV%" -u "%RAIZ_VM%\IA\agente_vm.py"
    goto :fim
)

set "PY_VM=C:\Users\a69613\AppData\Local\Programs\Python\Python311\python.exe"
if exist "%PY_VM%" (
    echo [3/4] Iniciando Agente com Python da VM: "%PY_VM%"
    echo [3/4] Iniciando Agente com Python da VM: "%PY_VM%" >> "%RAIZ_VM%\IA\vm_cmd_debug.txt"
    "%PY_VM%" -u "%RAIZ_VM%\IA\agente_vm.py"
    goto :fim
)

for /d %%d in ("%LOCALAPPDATA%\Programs\Python\Python*" "C:\Python*" "C:\Program Files\Python*") do (
    if exist "%%d\python.exe" (
        echo [3/4] Iniciando Agente com Python: "%%d\python.exe"
        echo [3/4] Iniciando Agente com Python: "%%d\python.exe" >> "%RAIZ_VM%\IA\vm_cmd_debug.txt"
        "%%d\python.exe" -u "%RAIZ_VM%\IA\agente_vm.py"
        goto :fim
    )
)

:fim
popd
pause
