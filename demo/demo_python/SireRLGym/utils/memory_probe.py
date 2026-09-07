"""Optional process-memory snapshots; never retain samples in memory."""

import ctypes
import json
import os
import sys
import time
from pathlib import Path


def process_memory_bytes():
    if sys.platform == "win32":
        from ctypes import wintypes

        class Counters(ctypes.Structure):
            _fields_ = [
                ("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                *[(name, ctypes.c_size_t) for name in (
                    "PeakWorkingSetSize", "WorkingSetSize",
                    "QuotaPeakPagedPoolUsage", "QuotaPagedPoolUsage",
                    "QuotaPeakNonPagedPoolUsage", "QuotaNonPagedPoolUsage",
                    "PagefileUsage", "PeakPagefileUsage", "PrivateUsage")],
            ]

        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        psapi = ctypes.WinDLL("psapi", use_last_error=True)
        kernel.GetCurrentProcess.restype = wintypes.HANDLE
        psapi.GetProcessMemoryInfo.argtypes = [
            wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD]
        psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
        counters = Counters()
        counters.cb = ctypes.sizeof(counters)
        if not psapi.GetProcessMemoryInfo(
                kernel.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
            raise ctypes.WinError(ctypes.get_last_error())
        return {"rss_bytes": counters.WorkingSetSize,
                "private_bytes": counters.PrivateUsage}
    if sys.platform.startswith("linux"):
        values = {}
        for line in Path("/proc/self/status").read_text().splitlines():
            key, _, value = line.partition(":")
            if key in ("VmRSS", "RssAnon", "VmSize"):
                values[key] = int(value.split()[0]) * 1024
        return {"rss_bytes": values.get("VmRSS"),
                "anonymous_bytes": values.get("RssAnon"),
                "virtual_bytes": values.get("VmSize")}
    raise RuntimeError("Memory sampling supports Windows and Linux")


def write_memory_sample(path, iteration, stage):
    sample = {"iteration": int(iteration), "stage": stage,
              "pid": os.getpid(), "time": time.time(), **process_memory_bytes()}
    with Path(path).open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(sample) + "\n")
