#!/usr/bin/env python3
"""Test to check if Vuer instance can be created and run"""
import sys
import os
import traceback

# Add televuer to path
televuer_root = os.path.join(os.path.dirname(__file__), "teleop", "televuer", "src")
sys.path.insert(0, televuer_root)

def test_vuer_instance():
    from vuer.server import Vuer
    import asyncio

    try:
        print("Creating Vuer instance...")
        vuer = Vuer(host='0.0.0.0',
                    cert='/home/user/myfiles/xr_teleoperate/teleop/televuer/cert.pem',
                    key='/home/user/myfiles/xr_teleoperate/teleop/televuer/key.pem',
                    queries=dict(grid=False), queue_len=3)
        print(f"✓ Vuer instance created: {vuer}")
        print(f"✓ Vuer type: {type(vuer)}")
        print(f"✓ Vuer.run: {vuer.run}")
        print(f"✓ Vuer.run type: {type(vuer.run)}")

        @vuer.add_handler("CAMERA_MOVE")
        async def on_cam_move(event, session, fps=60):
            print(f"Camera move event: {event}")

        print("✓ Handler added")

        async def main_app(session):
            print("Main app started")
            while True:
                await asyncio.sleep(1)

        vuer.spawn(main_app, start=False)
        print("✓ Spawn called")

        print("\nAttempting to call vuer.run()...")
        print(f"vuer.run is bound method: {isinstance(vuer.run.__self__, Vuer)}")

        # The actual call would start the server, so we won't actually call it here
        # vuer.run()

    except Exception as e:
        print(f"✗ Error: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    test_vuer_instance()
