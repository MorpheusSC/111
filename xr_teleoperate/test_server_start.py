#!/usr/bin/env python3
"""
Simple test to verify image server can start correctly
"""
import subprocess
import sys
import os
import time

print("=" * 60)
print("Testing Image Server Startup")
print("=" * 60)

# Get paths
project_root = os.getcwd()
start_script = os.path.join(project_root, "start_isaacsim_server.py")
log_file_path = os.path.join(project_root, "test_server.log")

print(f"Project root: {project_root}")
print(f"Start script: {start_script}")
print(f"Log file: {log_file_path}")
print(f"Python executable: {sys.executable}")

# Check if start script exists
if not os.path.exists(start_script):
    print(f"ERROR: Start script not found: {start_script}")
    sys.exit(1)

# Test aiortc import in current environment
print("\nTesting aiortc import...")
try:
    import aiortc
    print(f"✓ aiortc imported successfully (version {aiortc.__version__})")
except ImportError as e:
    print(f"✗ Failed to import aiortc: {e}")
    print("Please install: pip install aiortc")
    sys.exit(1)

# Start the server
print("\nStarting image server...")
log_file = open(log_file_path, "w")

env = os.environ.copy()
env["PATH"] = os.path.dirname(sys.executable) + ":" + env.get("PATH", "")

process = subprocess.Popen(
    [sys.executable, start_script, "--isaacsim"],
    stdout=log_file,
    stderr=subprocess.STDOUT,
    text=True,
    cwd=project_root,
    env=env
)

print(f"✓ Image server started with PID: {process.pid}")
print(f"  Log file: {log_file_path}")

# Wait a bit
print("\nWaiting 5 seconds for server to start...")
time.sleep(5.0)

# Check if still running
poll_result = process.poll()
if poll_result is None:
    print(f"✓ Server is still running (PID: {process.pid})")
else:
    print(f"✗ Server exited with code: {poll_result}")
    print("\nLog output:")
    with open(log_file_path, "r") as f:
        print(f.read())
    sys.exit(1)

# Read and display log
print("\nLast 30 lines of log:")
with open(log_file_path, "r") as f:
    lines = f.readlines()
    for line in lines[-30:]:
        print(line.rstrip())

print("\n" + "=" * 60)
print("✓ Server startup test PASSED")
print("=" * 60)

# Clean up
process.terminate()
process.wait(timeout=5)
print(f"\n✓ Server stopped")
