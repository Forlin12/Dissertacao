@echo off
pushd "%~dp0"

echo =================================================================
echo [1/4] Preparando Agente na Maquina Virtual...
echo Diretorio atual: "%CD%"
echo =================================================================

reg add HKCU\Console /v QuickEdit /t REG_DWORD /d 0 /f >nul 2>&1
powercfg /change standby-timeout-ac 0 >nul 2>&1
powercfg /change standby-timeout-dc 0 >nul 2>&1
powercfg /change hibernate-timeout-ac 0 >nul 2>&1
powercfg /change monitor-timeout-ac 0 >nul 2>&1

if exist ".git" (
    echo [2/4] Atualizando repositorio via Git...
    git pull --no-rebase
)

set "LOCAL_CACHE=%LOCALAPPDATA%\dissertacao_vm_cache"
if not exist "%LOCAL_CACHE%" mkdir "%LOCAL_CACHE%" >nul 2>&1
set "PYTHONPYCACHEPREFIX=%LOCAL_CACHE%"
set "NUMBA_CACHE_DIR=%LOCAL_CACHE%"
set "PYTHONUNBUFFERED=1"

echo [3/4] Localizando Python da Maquina Virtual...

set "PY_VENV=C:\Users\a69613\PycharmProjects\Dissertacao\.venv\Scripts\python.exe"
if exist "%PY_VENV%" (
    echo [4/4] Executando com Python VENV da VM: "%PY_VENV%"
    "%PY_VENV%" -u "IA\teste_todos_cenarios_vm.py"
    goto :fim
)

set "PY_VM=C:\Users\a69613\AppData\Local\Programs\Python\Python311\python.exe"
if exist "%PY_VM%" (
    echo [4/4] Executando com Python 3.11 da VM: "%PY_VM%"
    "%PY_VM%" -u "IA\teste_todos_cenarios_vm.py"
    goto :fim
)

set "PY_LOCAL311=%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
if exist "%PY_LOCAL311%" (
    echo [4/4] Executando com Python 3.11 Local: "%PY_LOCAL311%"
    "%PY_LOCAL311%" -u "IA\teste_todos_cenarios_vm.py"
    goto :fim
)

if exist ".venv\Scripts\python.exe" (
    echo [4/4] Executando com VENV local da pasta: ".venv\Scripts\python.exe"
    ".venv\Scripts\python.exe" -u "IA\teste_todos_cenarios_vm.py"
    goto :fim
)

echo [4/4] Executando com Python do PATH do Windows...
python -u "IA\teste_todos_cenarios_vm.py"

:fim
echo.
echo =================================================================
echo Execucao encerrada (ou interrompida). Pressione qualquer tecla...
echo =================================================================
pause
popd
