r"""
Agente leve que fica rodando na VM (APEREIRA-A69613) lendo a pasta compartilhada.
Blindado contra desligamento da VM:
  - Retoma automaticamente tarefas com status PENDENTE ou RODANDO (interrompidas por queda da VM)
  - Instala atalho na pasta Inicializar (Startup) da VM para reiniciar sozinho ao reconectar no RDP
"""
import os
import sys
import json
import time
import glob
import subprocess
import traceback

DIR_IA = os.path.dirname(os.path.abspath(__file__))
DIR_RAIZ = os.path.dirname(DIR_IA)
LOG_AGENTE = os.path.join(DIR_IA, "vm_agente.log")
FILA_ARQUIVO = os.path.join(DIR_IA, "vm_fila_comando.json")


def log_msg(msg):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    linha = f"[{ts}] {msg}"
    print(linha, flush=True)
    try:
        with open(LOG_AGENTE, "a", encoding="utf-8") as f:
            f.write(linha + "\n")
    except Exception:
        pass


def instalar_autostart_vm():
    """Garante que se a VM reiniciar e o usuário abrir a sessão RDP, o agente volte sozinho."""
    try:
        appdata = os.environ.get("APPDATA", "")
        if not appdata:
            return
        startup_dir = os.path.join(appdata, "Microsoft", "Windows", "Start Menu", "Programs", "Startup")
        if os.path.isdir(startup_dir):
            bat_startup = os.path.join(startup_dir, "iniciar_agente_dissertacao.bat")
            conteudo = (
                "@echo off\n"
                "if exist \"\\\\tsclient\\Z\\rodar_na_vm.bat\" (\n"
                "    call \"\\\\tsclient\\Z\\rodar_na_vm.bat\"\n"
                ") else if exist \"\\\\tsclient\\C\\Users\\Forlin\\Dissertação\\rodar_na_vm.bat\" (\n"
                "    call \"\\\\tsclient\\C\\Users\\Forlin\\Dissertação\\rodar_na_vm.bat\"\n"
                ")\n"
            )
            with open(bat_startup, "w", encoding="utf-8") as f:
                f.write(conteudo)
            log_msg(f"🔒 Autostart contra desligamento configurado em: {bat_startup}")
    except Exception as e:
        log_msg(f"ℹ️ Não foi possível gravar no Startup: {e}")


def encontrar_melhor_python():
    """Procura o Python da VM que já tenha geopandas/pygad/numba instalados (incluindo venvs do PyCharm)."""
    venv_direto = r"C:\Users\a69613\PycharmProjects\Dissertacao\.venv\Scripts\python.exe"
    if os.path.isfile(venv_direto):
        log_msg(f"✅ Python VENV principal encontrado instantaneamente: {venv_direto}")
        return venv_direto

    home = os.path.expanduser("~")
    candidatos = []
    for padrao in [
        os.path.join(home, "PycharmProjects", "*", ".venv", "Scripts", "python.exe"),
        os.path.join(home, "PycharmProjects", "*", "venv", "Scripts", "python.exe"),
        sys.executable,
    ]:
        for p in glob.glob(padrao):
            if os.path.isfile(p) and p not in candidatos:
                candidatos.append(p)

    if sys.executable not in candidatos:
        candidatos.append(sys.executable)

    log_msg(f"Candidatos Python encontrados na VM: {candidatos}")
    for py in candidatos:
        try:
            r = subprocess.run(
                [py, "-c", "import numpy, pygad, geopandas, numba; print('DEPS_OK')"],
                capture_output=True, text=True, timeout=25
            )
            if "DEPS_OK" in (r.stdout or ""):
                log_msg(f"✅ Python completo com todas as bibliotecas encontrado: {py}")
                return py
            else:
                log_msg(f"ℹ️ {py} saída: {(r.stderr or r.stdout or '').strip()[:200]}")
        except Exception as e:
            log_msg(f"⚠️ Erro ao testar {py}: {e}")

    return sys.executable


def _iniciar_keepalive_rdp():
    """Envia pulso invisível a cada 45s para impedir desconexão de sessão RDP por ociosidade (GPO de 1 hora)."""
    import threading
    def _loop():
        while True:
            try:
                import ctypes
                ctypes.windll.kernel32.SetThreadExecutionState(0x80000003)
                ctypes.windll.user32.mouse_event(0x0001, 0, 0, 0, 0)
            except Exception:
                pass
            time.sleep(45)
    t = threading.Thread(target=_loop, daemon=True)
    t.start()


def main():
    _iniciar_keepalive_rdp()
    try:
        import ctypes
        # Impede suspensão do Windows
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000003)
        # Desativa o QuickEdit Mode do CMD do Windows para que um clique na janela preta NUNCA congele o script!
        kernel32 = ctypes.windll.kernel32
        h_stdin = kernel32.GetStdHandle(-10)
        mode = ctypes.c_uint32()
        if kernel32.GetConsoleMode(h_stdin, ctypes.byref(mode)):
            # Remove ENABLE_QUICK_EDIT_MODE (0x0040) e adiciona ENABLE_EXTENDED_FLAGS (0x0080)
            novo_mode = (mode.value & ~0x0040) | 0x0080
            kernel32.SetConsoleMode(h_stdin, novo_mode)
    except Exception:
        pass
    log_msg(f"🤖 Agente VM iniciado em {os.environ.get('COMPUTERNAME')} (CPUs: {os.cpu_count()})")
    instalar_autostart_vm()
    melhor_py = encontrar_melhor_python()

    # Se não existir fila, cria a tarefa inicial para rodar todos os cenários
    if not os.path.exists(FILA_ARQUIVO):
        with open(FILA_ARQUIVO, "w", encoding="utf-8") as f:
            json.dump({"id": 1, "script": "IA/teste_todos_cenarios_vm.py", "status": "PENDENTE"}, f, indent=2)

    ultimo_id_concluido = -1
    primeira_iteracao = True
    log_msg("📡 Aguardando e processando tarefas automaticamente (deixe esta janela aberta)...")

    while True:
        try:
            if os.path.exists(FILA_ARQUIVO):
                with open(FILA_ARQUIVO, "r", encoding="utf-8") as f:
                    tarefa = json.load(f)

                tid = tarefa.get("id", 0)
                status = tarefa.get("status", "")
                # Se o agente acabou de subir após queda da VM e a tarefa estava RODANDO, retoma automaticamente!
                deve_rodar = (status == "PENDENTE" and tid != ultimo_id_concluido) or (
                    primeira_iteracao and status == "RODANDO"
                )
                primeira_iteracao = False

                if deve_rodar:
                    script_rel = tarefa.get("script", "IA/teste_todos_cenarios_vm.py")
                    args_extra = tarefa.get("args", [])
                    caminho_script = os.path.join(DIR_RAIZ, script_rel.replace("/", os.sep))

                    tarefa["status"] = "RODANDO"
                    tarefa["inicio"] = time.strftime("%Y-%m-%d %H:%M:%S")
                    tarefa["python_usado"] = melhor_py
                    with open(FILA_ARQUIVO, "w", encoding="utf-8") as f:
                        json.dump(tarefa, f, indent=2)

                    log_msg(f"▶️ Executando tarefa #{tid}: {melhor_py} -u {caminho_script} {' '.join(args_extra)}")
                    env = os.environ.copy()
                    local_cache = os.path.join(os.environ.get("LOCALAPPDATA", "C:\\Temp"), "dissertacao_vm_cache")
                    os.makedirs(local_cache, exist_ok=True)
                    env["NUMBA_CACHE_DIR"] = local_cache
                    env["PYTHONPYCACHEPREFIX"] = local_cache
                    env["PYTHONUNBUFFERED"] = "1"
                    proc = subprocess.run(
                        [melhor_py, "-u", caminho_script] + list(args_extra),
                        cwd=DIR_RAIZ, env=env
                    )
                    if proc.returncode == 0:
                        tarefa["status"] = "CONCLUIDO"
                        ultimo_id_concluido = tid
                    else:
                        tarefa["status"] = f"ERRO_{proc.returncode}"
                        ultimo_id_concluido = tid
                    tarefa["fim"] = time.strftime("%Y-%m-%d %H:%M:%S")
                    with open(FILA_ARQUIVO, "w", encoding="utf-8") as f:
                        json.dump(tarefa, f, indent=2)
                    log_msg(f"⏹️ Tarefa #{tid} finalizada com status: {tarefa['status']}")
        except Exception as e:
            log_msg(f"⚠️ Erro no loop do agente: {e}\n{traceback.format_exc()}")

        time.sleep(3)


if __name__ == "__main__":
    main()
