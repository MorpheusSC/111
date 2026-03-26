#!/usr/bin/env python3
"""Test if shared memory can be accessed in subprocess"""
import sys
import os
from multiprocessing import Process, Array
import time

class TestSharedMemory:
    def __init__(self):
        print(f"Parent: Creating shared memory")
        self.shared = Array('d', 16, lock=True)
        print(f"Parent: Shared memory created at {id(self.shared)}")

        self.process = Process(target=self._subprocess_task)
        self.process.daemon = True
        self.process.start()

    def _subprocess_task(self):
        print(f"Subprocess: Started")
        print(f"Subprocess: self.shared at {id(self.shared)}")

        try:
            print(f"Subprocess: Reading from shared memory...")
            with self.shared.get_lock():
                data = self.shared[:]
                print(f"Subprocess: Read data: {data[:5]}...")

            print(f"Subprocess: Writing to shared memory...")
            with self.shared.get_lock():
                for i in range(16):
                    self.shared[i] = float(i)

            print(f"Subprocess: Success!")

        except Exception as e:
            print(f"Subprocess: Error: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    print("=" * 80)
    print("Testing shared memory in subprocess")
    print("=" * 80)

    obj = TestSharedMemory()

    time.sleep(2)

    print("\nParent: Checking shared memory...")
    with obj.shared.get_lock():
        data = obj.shared[:]
        print(f"Parent: Read data: {data}")

    print(f"\nSubprocess status: {'Running' if obj.process.is_alive() else 'Exited'}")
    if not obj.process.is_alive():
        print(f"Exit code: {obj.process.exitcode}")
