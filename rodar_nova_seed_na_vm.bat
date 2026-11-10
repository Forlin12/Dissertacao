@echo off
chcp 65001 >nul
reg add HKCU\Console /v QuickEdit /t REG_DWORD /d 0 /f >nul 2>&1

set "PASTA_SCRIPT=%~dp0"
if "%PASTA_SCRIPT:~-1%"=="\" set "PASTA_SCRIPT=%PASTA_SCRIPT:~0,-1%"
pushd "%PASTA_SCRIPT%" 2>nul

set "RAIZ_VM=%CD%"
if "%RAIZ_VM:~-1%"=="\" set "RAIZ_VM=%RAIZ_VM:~0,-1%"

echo [1/4] Configurando ambiente na VM...
taskkill /F /IM python.exe /T >nul 2>&1
powercfg /change standby-timeout-ac 0 >nul 2>&1
powercfg /change standby-timeout-dc 0 >nul 2>&1
powercfg /change hibernate-timeout-ac 0 >nul 2>&1
powercfg /change monitor-timeout-ac 0 >nul 2>&1

if exist "%RAIZ_VM%\.git" (
    echo [2/4] Atualizando repositorio via Git...
    git -C "%RAIZ_VM%" pull --no-rebase 2>nul
)

set "LOCAL_CACHE=%LOCALAPPDATA%\dissertacao_vm_cache"
if not exist "%LOCAL_CACHE%" mkdir "%LOCAL_CACHE%" >nul 2>&1
set "PYTHONPYCACHEPREFIX=%LOCAL_CACHE%"
set "NUMBA_CACHE_DIR=%LOCAL_CACHE%"
set "PYTHONUNBUFFERED=1"

cd /d "%RAIZ_VM%" 2>nul

echo [3/4] Localizando Python da VM...

if exist "%RAIZ_VM%\.venv\Scripts\python.exe" (
    echo [4/4] Iniciando os 9 Cenarios com Python VENV do Workspace: "%RAIZ_VM%\.venv\Scripts\python.exe" %*
    "%RAIZ_VM%\.venv\Scripts\python.exe" -u "%RAIZ_VM%\IA\teste_nova_seed_vm.py" %*
    goto :fim
)

set "PY_VENV=C:\Users\a69613\PycharmProjects\Dissertacao\.venv\Scripts\python.exe"
if exist "%PY_VENV%" (
    echo [4/4] Iniciando os 9 Cenarios com: "%PY_VENV%" %*
    "%PY_VENV%" -u "%RAIZ_VM%\IA\teste_nova_seed_vm.py" %*
    goto :fim
)

set "PY_VM=C:\Users\a69613\AppData\Local\Programs\Python\Python311\python.exe"
if exist "%PY_VM%" (
    echo [4/4] Iniciando os 9 Cenarios com: "%PY_VM%" %*
    "%PY_VM%" -u "%RAIZ_VM%\IA\teste_nova_seed_vm.py" %*
    goto :fim
)

for /d %%d in ("%USERPROFILE%\PycharmProjects\*\venv" "%USERPROFILE%\PycharmProjects\*\.venv") do (
    if exist "%%d\Scripts\python.exe" (
        echo [4/4] Iniciando os 9 Cenarios com VENV encontrado: "%%d\Scripts\python.exe" %*
        "%%d\Scripts\python.exe" -u "%RAIZ_VM%\IA\teste_nova_seed_vm.py" %*
        goto :fim
    )
)

for /d %%d in ("%LOCALAPPDATA%\Programs\Python\Python*" "C:\Python*" "C:\Program Files\Python*") do (
    if exist "%%d\python.exe" (
        echo [4/4] Iniciando os 9 Cenarios com Python: "%%d\python.exe" %*
        "%%d\python.exe" -u "%RAIZ_VM%\IA\teste_nova_seed_vm.py" %*
        goto :fim
    )
)

python -u "%RAIZ_VM%\IA\teste_nova_seed_vm.py" %*

:fim
popd
echo.
echo Execucao finalizada. Pressione qualquer tecla para sair.
pause >nul
