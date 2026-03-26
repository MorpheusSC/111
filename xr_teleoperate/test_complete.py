#!/usr/bin/env python3
"""
Complete test to reproduce the issue with TeleVuer
"""
import sys
import os

# Add televuer to path
televuer_root = os.path.join(os.path.dirname(__file__), "teleop", "televuer", "src")
sys.path.insert(0, televuer_root)

import time
import traceback

def test_televuer_direct():
    """Test TeleVuer directly"""
    print("=" * 80)
    print("Testing TeleVuer (not TeleVuerWrapper)")
    print("=" * 80)

    try:
        from televuer import TeleVuer

        # Create TeleVuer instance
        print("\nCreating TeleVuer instance...")
        tv = TeleVuer(
            use_hand_tracking=False,
            binocular=True,
            img_shape=(480, 1280),
            display_fps=30.0,
            display_mode="pass-through",  # Simple mode, no image transmission needed
            zmq=False,
            webrtc=False
        )
        print("✓ TeleVuer instance created")

        # Wait for subprocess to start
        time.sleep(3)

        # Check process status
        print(f"\nSubprocess status: {'Running' if tv.process.is_alive() else 'Exited'}")
        if not tv.process.is_alive():
            print(f"Exit code: {tv.process.exitcode}")
        else:
            print("✓ Subprocess is running")
            print("\nAccess https://localhost:8012 to test connection")
            print("Press Ctrl+C to stop...")

        # Keep running
        try:
            while tv.process.is_alive():
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping...")
            tv.close()

        print("Test completed")

    except Exception as e:
        print(f"✗ Error: {e}")
        traceback.print_exc()

def test_televuer_wrapper():
    """Test TeleVuerWrapper"""
    print("\n" + "=" * 80)
    print("Testing TeleVuerWrapper")
    print("=" * 80)

    try:
        from televuer import TeleVuerWrapper

        # Create TeleVuerWrapper instance
        print("\nCreating TeleVuerWrapper instance...")
        tvw = TeleVuerWrapper(
            use_hand_tracking=False,
            binocular=True,
            img_shape=(480, 1280),
            display_fps=30.0,
            display_mode="pass-through",
            zmq=False,
            webrtc=False
        )
        print("✓ TeleVuerWrapper instance created")

        # Wait for subprocess to start
        time.sleep(3)

        # Check process status
        print(f"\nSubprocess status: {'Running' if tvw.tvuer.process.is_alive() else 'Exited'}")
        if not tvw.tvuer.process.is_alive():
            print(f"Exit code: {tvw.tvuer.process.exitcode}")
        else:
            print("✓ Subprocess is running")
            print("\nAccess https://localhost:8012 to test connection")
            print("Press Ctrl+C to stop...")

        # Keep running
        try:
            while tvw.tvuer.process.is_alive():
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping...")
            tvw.tvuer.close()

        print("Test completed")

    except Exception as e:
        print(f"✗ Error: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    # Test 1: TeleVuer
    test_televuer_direct()

    # Clean up
    time.sleep(1)

    # Test 2: TeleVuerWrapper
    test_televuer_wrapper()
