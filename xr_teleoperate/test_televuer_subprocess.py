#!/usr/bin/env python3
"""Test to reproduce the TeleVuer subprocess issue"""
import sys
import os
import asyncio
from multiprocessing import Process, Array
import time

# Add televuer to path
televuer_root = os.path.join(os.path.dirname(__file__), "teleop", "televuer", "src")
sys.path.insert(0, televuer_root)

class TestVuerWrapper:
    def __init__(self):
        # Create shared memory in parent process
        self.head_pose_shared = Array('d', 16, lock=True)
        print(f"✓ Parent: Created head_pose_shared at {id(self.head_pose_shared)}")
        
        # Save cert paths for subprocess
        self._cert_file = '/home/user/myfiles/xr_teleoperate/teleop/televuer/cert.pem'
        self._key_file = '/home/user/myfiles/xr_teleoperate/teleop/televuer/key.pem'
        
        # Create process
        self.process = Process(target=self._vuer_run)
        self.process.daemon = True
        self.process.start()
    
    def _vuer_run(self):
        """This runs in subprocess"""
        print(f"✓ Subprocess: Started")
        print(f"✓ Subprocess: head_pose_shared at {id(self.head_pose_shared)}")
        
        try:
            from vuer.server import Vuer
            print("✓ Subprocess: Vuer imported")
            
            # Create Vuer instance
            vuer = Vuer(host='0.0.0.0', cert=self._cert_file, key=self._key_file,
                        queries=dict(grid=False), queue_len=3)
            print("✓ Subprocess: Vuer instance created")
            
            # Add handler that uses shared memory
            @vuer.add_handler("CAMERA_MOVE")
            async def on_cam_move(event, session, fps=60):
                try:
                    print(f"✓ Subprocess: Handler called, accessing shared memory")
                    with self.head_pose_shared.get_lock():
                        self.head_pose_shared[:] = event.value["camera"]["matrix"]
                        print(f"✓ Subprocess: Successfully wrote to shared memory")
                except Exception as e:
                    print(f"✗ Subprocess: Handler error: {e}")
                    import traceback
                    traceback.print_exc()
            
            print("✓ Subprocess: Handler added")
            
            # Define spawn function
            async def main_app(session):
                print("✓ Subprocess: Main app started!")
                while True:
                    await asyncio.sleep(1)
            
            import asyncio
            vuer.spawn(main_app, start=False)
            print("✓ Subprocess: Spawn called")
            
            # Run Vuer
            print("✓ Subprocess: Calling vuer.run()...")
            vuer.run()
            
        except Exception as e:
            print(f"✗ Subprocess: Error: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    print("=" * 80)
    print("Testing TeleVuer-like subprocess issue")
    print("=" * 80)
    
    wrapper = TestVuerWrapper()
    
    print("\nWaiting for subprocess to start...")
    time.sleep(3)
    
    print(f"\nSubprocess status: {'Running' if wrapper.process.is_alive() else 'Exited'}")
    if not wrapper.process.is_alive():
        print(f"Exit code: {wrapper.process.exitcode}")
    
    print("\nAccess https://localhost:8012 to test connection")
    print("Press Ctrl+C to stop...")
    
    try:
        while wrapper.process.is_alive():
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
        wrapper.process.terminate()
        wrapper.process.join()
    
    print("Done")
