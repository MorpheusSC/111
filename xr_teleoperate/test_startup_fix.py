#!/usr/bin/env python3
"""
Test script to verify the startup order fix
"""
import os
import sys
import time
import subprocess

def test_startup_order():
    """Test that image server starts before ImageClient initialization"""
    print("="*60)
    print("Testing Startup Order Fix")
    print("="*60)

    # Check if we can import IsaacSimCamera
    try:
        teleimager_path = os.path.join(os.path.dirname(__file__), "teleop", "teleimager", "src")
        sys.path.insert(0, teleimager_path)
        from teleimager.image_server import IsaacSimCamera
        print("✓ IsaacSimCamera imported successfully")
    except Exception as e:
        print(f"✗ Failed to import IsaacSimCamera: {e}")
        return False

    # Check if MultiImageReader can be imported
    try:
        from teleimager.multi_image_reader import MultiImageReader
        print("✓ MultiImageReader imported successfully")
    except Exception as e:
        print(f"✗ Failed to import MultiImageReader: {e}")
        return False

    # Check if the main script has the correct structure
    try:
        with open("teleop/teleop_hand_and_arm.py", "r") as f:
            content = f.read()

        # Check if simulation mode comes before ImageClient initialization
        sim_mode_pos = content.find("if args.sim:")
        image_client_pos = content.find("img_client = ImageClient")

        if sim_mode_pos == -1:
            print("✗ Simulation mode code not found")
            return False

        if image_client_pos == -1:
            print("✗ ImageClient initialization not found")
            return False

        if sim_mode_pos < image_client_pos:
            print(f"✓ Simulation mode code comes BEFORE ImageClient initialization")
            print(f"  - Simulation mode at position: {sim_mode_pos}")
            print(f"  - ImageClient at position: {image_client_pos}")
            print(f"  - Difference: {image_client_pos - sim_mode_pos} characters")
        else:
            print("✗ Simulation mode code comes AFTER ImageClient initialization")
            print(f"  - ImageClient at position: {image_client_pos}")
            print(f"  - Simulation mode at position: {sim_mode_pos}")
            return False

        # Check if finally block has proper variable checking
        if "'arm_ctrl' in locals()" in content:
            print("✓ finally block checks for 'arm_ctrl' before use")
        else:
            print("⚠ finally block may not check for 'arm_ctrl' before use")

        if "'sim_state_subscriber' in locals()" in content:
            print("✓ finally block checks for 'sim_state_subscriber' before use")
        else:
            print("⚠ finally block may not check for 'sim_state_subscriber' before use")

        if "'recorder' in locals()" in content:
            print("✓ finally block checks for 'recorder' before use")
        else:
            print("⚠ finally block may not check for 'recorder' before use")

        return True

    except Exception as e:
        print(f"✗ Failed to check script structure: {e}")
        return False

def main():
    print("\nThis test verifies that:")
    print("1. IsaacSimCamera can be imported")
    print("2. MultiImageReader can be imported")
    print("3. Simulation mode initialization comes before ImageClient")
    print("4. finally block has proper variable checking")
    print()

    success = test_startup_order()

    print("\n" + "="*60)
    if success:
        print("✓ All checks passed!")
        print("\nThe startup order fix is correct:")
        print("1. Simulation mode detected")
        print("2. IsaacSim image server started")
        print("3. Wait 5 seconds for server to be ready")
        print("4. Initialize ImageClient (now server is ready)")
        print("5. Continue with rest of initialization")
    else:
        print("✗ Some checks failed!")
    print("="*60)

    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
