import logging
import os
import signal
import subprocess
import sys
import ctypes
from pathlib import Path
from typing import Union

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class ProgramRunner:
    def __init__(self):
        self.log = logging.getLogger(__name__)
        self.process: subprocess.Popen | None = None
        self._is_paused = False

    def run(self, script_path: Path, repeat: Union[int, float, str] = 1,
            host: str = "localhost",
            push_port: int = 5555,
            sub_port: int = 5556,
            req_port: int = 5557) -> bool:
        self.stop()
        if not script_path.exists():
            self.log.error(f"Program not found: {script_path}")
            return False

        env = os.environ.copy()
        env["PYTHONPATH"] = str(PROJECT_ROOT) + os.pathsep + env.get("PYTHONPATH", "")

        self.log.info(f"Starting program process: {script_path}")
        self._is_paused = False
        self.process = subprocess.Popen([
            sys.executable,
            str(script_path),
            "--repeat", str(repeat),
            "--host", host,
            "--push-port", str(push_port),
            "--sub-port", str(sub_port),
            "--req-port", str(req_port),
            ],
            env=env
        )
        return True

    def pause(self) -> None:
        if self.process is not None and self.process.poll() is None and not self._is_paused:
            self.log.info(f"Suspending process tree for PID {self.process.pid}...")
            if sys.platform == "win32":
                self._control_win32_process_tree(suspend=True)
            else:
                os.kill(self.process.pid, signal.SIGSTOP)
            self._is_paused = True

    def resume(self) -> None:
        if self.process is not None and self.process.poll() is None and self._is_paused:
            self.log.info(f"Resuming process tree for PID {self.process.pid}...")
            if sys.platform == "win32":
                self._control_win32_process_tree(suspend=False)
            else:
                os.kill(self.process.pid, signal.SIGCONT)
            self._is_paused = False

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.log.info("Stopping program process...")
            if self._is_paused:
                self.resume()
            self.process.kill()
            self.process.wait()
            self.log.info("Program process stopped.")
        self.process = None
        self._is_paused = False

    def _control_win32_process_tree(self, suspend: bool) -> None:
        """Suspend/Resume parent process AND all spawned child processes via Windows API."""
        try:
            # Get list of all PIDs in the tree (parent + children)
            pids = self._get_win32_child_pids(self.process.pid)
            pids.append(self.process.pid)

            for pid in pids:
                # PROCESS_SUSPEND_RESUME = 0x0800
                handle = ctypes.windll.kernel32.OpenProcess(0x0800, False, pid)
                if handle:
                    if suspend:
                        ctypes.windll.ntdll.NtSuspendProcess(handle)
                    else:
                        ctypes.windll.ntdll.NtResumeProcess(handle)
                    ctypes.windll.kernel32.CloseHandle(handle)
        except Exception as e:
            self.log.error(f"Failed to control process tree: {e}")

    def _get_win32_child_pids(self, parent_pid: int) -> list[int]:
        """Find all child process IDs using Windows Toolhelp32 Snapshot."""
        child_pids = []
        try:
            # TH32CS_SNAPPROCESS = 0x00000002
            h_snapshot = ctypes.windll.kernel32.CreateToolhelp32Snapshot(0x02, 0)
            if h_snapshot == -1:
                return child_pids

            class PROCESSENTRY32(ctypes.Structure):
                _fields_ = [
                    ("dwSize", ctypes.c_ulong),
                    ("cntUsage", ctypes.c_ulong),
                    ("th32ProcessID", ctypes.c_ulong),
                    ("th32DefaultHeapID", ctypes.c_size_t),
                    ("th32ModuleID", ctypes.c_ulong),
                    ("cntThreads", ctypes.c_ulong),
                    ("th32ParentProcessID", ctypes.c_ulong),
                    ("pcPriClassBase", ctypes.c_long),
                    ("dwFlags", ctypes.c_ulong),
                    ("szExeFile", ctypes.c_char * 260)
                ]

            pe = PROCESSENTRY32()
            pe.dwSize = ctypes.sizeof(PROCESSENTRY32)

            if ctypes.windll.kernel32.Process32First(h_snapshot, ctypes.byref(pe)):
                while True:
                    if pe.th32ParentProcessID == parent_pid:
                        child_pids.append(pe.th32ProcessID)
                    if not ctypes.windll.kernel32.Process32Next(h_snapshot, ctypes.byref(pe)):
                        break

            ctypes.windll.kernel32.CloseHandle(h_snapshot)
        except Exception as e:
            self.log.error(f"Error snapshotting child processes: {e}")
        return child_pids
