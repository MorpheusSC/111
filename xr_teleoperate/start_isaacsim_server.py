#!/usr/bin/env python3
"""
Standalone script to start IsaacSim image server
"""
import sys
import os

# Debug: Print Python executable and path
print(f"[DEBUG] Python executable: {sys.executable}")
print(f"[DEBUG] Python version: {sys.version}")
print(f"[DEBUG] Python path: {sys.path[:3]}...")

# Add teleimager src to path
script_dir = os.path.dirname(os.path.abspath(__file__))
teleimager_src = os.path.join(script_dir, "teleop", "teleimager", "src")
sys.path.insert(0, teleimager_src)
print(f"[DEBUG] Added to path: {teleimager_src}")

# Try importing aiortc first
try:
    import aiortc
    print(f"[DEBUG] aiortc imported successfully, version: {aiortc.__version__}")
except ImportError as e:
    print(f"[ERROR] Failed to import aiortc: {e}")
    print(f"[ERROR] Please install aiortc: pip install aiortc")
    sys.exit(1)

# Import and run main function
try:
    from teleimager.image_server import main
    print("[DEBUG] main function imported successfully")
except Exception as e:
    print(f"[ERROR] Failed to import main: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

if __name__ == "__main__":
    # Pass command line arguments to main
    sys.argv = [sys.argv[0]] + sys.argv[1:]
    print(f"[DEBUG] Starting main with args: {sys.argv}")
    main()
