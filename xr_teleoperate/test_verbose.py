#!/usr/bin/env python3
"""
Test TeleVuer with verbose output to catch any errors
"""
import sys
import os

# Add televuer to path
televuer_root = os.path.join(os.path.dirname(__file__), "teleop", "televuer", "src")
sys.path.insert(0, televuer_root)

import time
import traceback
import logging
import logging_mp

# Configure verbose logging
logging.basicConfig(level=logging.DEBUG)
logging_mp.basicConfig(level=logging_mp.DEBUG)

def test_with_verbose_output():
    """Test with verbose output"""
    print("=" * 80)
    print("TeleVuer Verbose Test")
    print("=" * 80)

    try:
        from televuer import TeleVuer

        # Create TeleVuer instance with pass-through mode (simplest)
        print("\n[1] Creating TeleVuer instance...")
        tv = TeleVuer(
            use_hand_tracking=False,
            binocular=True,
            img_shape=(480, 1280),
            display_fps=30.0,
            display_mode="pass-through",
            zmq=False,
            webrtc=False
        )
        print("[✓] TeleVuer instance created")

        # Wait for subprocess
        print("\n[2] Waiting for subprocess to initialize...")
        time.sleep(2)

        # Check status
        print(f"[3] Subprocess status: {'Running' if tv.process.is_alive() else 'Exited'}")
        if not tv.process.is_alive():
            print(f"[!] Exit code: {tv.process.exitcode}")
            print("[✗] Subprocess exited early!")
            return
        else:
            print("[✓] Subprocess is running")

        print("\n[4] Testing for 5 seconds...")
        print("[INFO] Access https://localhost:8012 with your headset")
        print("[INFO] Try refreshing and check for any errors in this terminal")

        for i in range(5):
            if not tv.process.is_alive():
                print(f"\n[!] Subprocess died at second {i+1}!")
                print(f"[!] Exit code: {tv.process.exitcode}")
                break
            print(f"[{i+1}/5] Subprocess still running...")
            time.sleep(1)

        print("\n[5] Checking final status...")
        if tv.process.is_alive():
            print("[✓] Subprocess is still running!")
            print("[✓] No errors detected")
        else:
            print(f"[✗] Subprocess exited")
            print(f"[✗] Exit code: {tv.process.exitcode}")

        print("\n[6] Cleaning up...")
        tv.close()
        print("[✓] Cleanup completed")

    except Exception as e:
        print(f"\n[✗] Error during test: {e}")
        print(f"\n[!] Full traceback:")
        traceback.print_exc()

if __name__ == "__main__":
    test_with_verbose_output()
