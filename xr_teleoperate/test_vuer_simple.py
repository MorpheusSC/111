#!/usr/bin/env python3
"""Simple test to diagnose Vuer subprocess issue"""
import sys
import os

# Add televuer to path
televuer_root = os.path.join(os.path.dirname(__file__), "teleop", "televuer", "src")
sys.path.insert(0, televuer_root)

import time
import traceback
from multiprocessing import Process

def simple_vuer_test():
    """Test Vuer directly without TeleVuer wrapper"""
    try:
        from vuer.server import Vuer
        print("✓ Vuer imported successfully")
        
        # Create Vuer instance
        vuer = Vuer(host='0.0.0.0', cert='/home/user/myfiles/xr_teleoperate/teleop/televuer/cert.pem',
                    key='/home/user/myfiles/xr_teleoperate/teleop/televuer/key.pem',
                    queries=dict(grid=False), queue_len=3)
        print("✓ Vuer instance created")
        
        # Define a simple handler
        @vuer.add_handler("CAMERA_MOVE")
        async def on_cam_move(event, session, fps=60):
            print(f"Camera move event received: {event}")
        
        print("✓ Handler added")
        
        # Define spawn function
        async def main_app(session):
            print("Main app started!")
            while True:
                await asyncio.sleep(1)
        
        import asyncio
        vuer.spawn(main_app, start=False)
        print("✓ Spawn called")
        
        # Try to run
        print("Attempting to run Vuer...")
        vuer.run()
        
    except Exception as e:
        print(f"✗ Error in simple_vuer_test: {e}")
        traceback.print_exc()

def run_in_subprocess():
    """Test running in subprocess"""
    print("Testing Vuer in subprocess...")
    p = Process(target=simple_vuer_test)
    p.start()
    
    # Wait a bit
    time.sleep(3)
    
    # Check if process is still running
    if p.is_alive():
        print("✓ Vuer process is running!")
        print("Check if you can access https://localhost:8012")
        input("Press Enter to stop...")
        p.terminate()
        p.join()
    else:
        print(f"✗ Vuer process exited with code: {p.exitcode}")

if __name__ == "__main__":
    print("=" * 80)
    print("Simple Vuer Test")
    print("=" * 80)
    
    # Test 1: Run directly
    print("\nTest 1: Running Vuer directly in this process")
    print("Press Ctrl+C to stop this test\n")
    try:
        simple_vuer_test()
    except KeyboardInterrupt:
        print("\nTest 1 stopped by user")
    except Exception as e:
        print(f"Test 1 failed: {e}")
    
    print("\n" + "=" * 80)
    print("Test 2: Running Vuer in subprocess")
    print("=" * 80)
    run_in_subprocess()
