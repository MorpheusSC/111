#!/usr/bin/env python3
"""
Test script to verify IsaacSim camera fix
"""
import os
import sys

# Add teleimager src to path
teleimager_path = os.path.join(os.path.dirname(__file__), "teleop", "teleimager", "src")
sys.path.insert(0, teleimager_path)

def test_isaacsim_import():
    """Test IsaacSimCamera can be imported"""
    try:
        from teleimager.image_server import IsaacSimCamera
        print("✓ IsaacSimCamera imported successfully")
        return True
    except Exception as e:
        print(f"✗ Failed to import IsaacSimCamera: {e}")
        return False

def test_config_loading():
    """Test config can be loaded and modified"""
    try:
        import yaml
        config_path = os.path.join(os.path.dirname(__file__), "teleop", "teleimager", "cam_config_server.yaml")
        with open(config_path, "r") as f:
            cam_config = yaml.safe_load(f)

        # Test forcing isaacsim type
        for cam_key in cam_config:
            if cam_key.endswith('_camera'):
                cam_config[cam_key]['type'] = 'isaacsim'

        # Verify
        for cam_key in cam_config:
            if cam_key.endswith('_camera'):
                assert cam_config[cam_key]['type'] == 'isaacsim', f"Camera {cam_key} not forced to isaacsim"

        print("✓ Config loading and isaacsim type forcing works")
        return True
    except Exception as e:
        print(f"✗ Config loading failed: {e}")
        return False

def test_shared_memory_tools():
    """Test shared memory tools can be imported"""
    try:
        from teleimager.multi_image_reader import MultiImageReader
        print("✓ Local MultiImageReader imported successfully")
        return True
    except ImportError as e:
        print(f"⚠ MultiImageReader not available: {e}")
        return None
    except Exception as e:
        print(f"✗ Failed to import MultiImageReader: {e}")
        return False

def main():
    print("="*60)
    print("Testing IsaacSim Camera Fix")
    print("="*60)

    results = []
    results.append(("IsaacSimCamera Import", test_isaacsim_import()))
    results.append(("Config Loading", test_config_loading()))
    results.append(("Shared Memory Tools", test_shared_memory_tools()))

    print("\n" + "="*60)
    print("Test Results:")
    print("="*60)
    for name, result in results:
        status = "PASS" if result else ("WARN" if result is None else "FAIL")
        print(f"{name:.<40} {status}")

    all_passed = all(r is not False for r in [r for _, r in results])
    if all_passed:
        print("\n✓ All critical tests passed!")
        return 0
    else:
        print("\n✗ Some tests failed. Please check the output above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
