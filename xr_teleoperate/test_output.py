#!/usr/bin/env python3
"""Test multiprocessing output"""
import sys
import os
import time
from multiprocessing import Process

def subprocess_task():
    print("Subprocess: Hello from subprocess!", flush=True)
    sys.stdout.flush()
    time.sleep(1)
    print("Subprocess: Still here!", flush=True)
    sys.stderr.write("Subprocess: Error message\n")
    sys.stderr.flush()
    time.sleep(2)
    print("Subprocess: Exiting...")

if __name__ == "__main__":
    print("Main: Starting subprocess...")
    p = Process(target=subprocess_task)
    p.start()
    print(f"Main: Subprocess started, PID={p.pid}")
    
    time.sleep(0.5)
    print("Main: Waiting for subprocess...")
    
    p.join()
    print(f"Main: Subprocess exited with code {p.exitcode}")
