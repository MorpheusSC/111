"""
MultiImageReader implementation for reading IsaacSim camera data from shared memory.
This is a simplified version that can work without the full unitree_sim_isaaclab library.
"""

import logging_mp
logger_mp = logging_mp.getLogger(__name__)
from multiprocessing import shared_memory
import numpy as np
import time


class MultiImageReader:
    """
    Read images from shared memory published by IsaacSim.
    
    Expected shared memory layout for each camera:
    - First 8 bytes: timestamp (4 bytes) + data length (4 bytes)
    - Following bytes: image data (JPEG encoded)
    """
    
    def __init__(self):
        """Initialize the image reader."""
        self.shm_cams = {}
        self.image_shapes = {
            'head': (720, 1280, 3),  # Default shape for head camera
            'left': (720, 1280, 3),  # Default shape for left camera
            'right': (720, 1280, 3)  # Default shape for right camera
        }
        self.max_buffer_size = 10 * 1024 * 1024  # 10MB max buffer per camera
        logger_mp.info("[MultiImageReader] Initialized")
    
    def _get_shm_name(self, source: str) -> str:
        """Get shared memory name for a camera source."""
        return f"isaacsim_camera_{source}"
    
    def _connect_shm(self, source: str) -> bool:
        """Connect to shared memory for a camera source."""
        if source in self.shm_cams:
            return True
        
        shm_name = self._get_shm_name(source)
        try:
            shm = shared_memory.SharedMemory(name=shm_name)
            self.shm_cams[source] = shm
            logger_mp.info(f"[MultiImageReader] Connected to shared memory: {shm_name}")
            return True
        except FileNotFoundError:
            logger_mp.debug(f"[MultiImageReader] Shared memory not found: {shm_name} (IsaacSim may not be running or camera not enabled)")
            return False
        except Exception as e:
            logger_mp.warning(f"[MultiImageReader] Failed to connect to shared memory {shm_name}: {e}")
            return False
    
    def read_single_image(self, source: str) -> np.ndarray:
        """
        Read a single image from shared memory.
        
        Args:
            source: camera source ('head', 'left', or 'right')
            
        Returns:
            numpy array: decoded image as BGR format, or None if no data available
        """
        if source not in self.image_shapes:
            logger_mp.warning(f"[MultiImageReader] Unknown source: {source}")
            return None
        
        # Try to connect if not already connected
        if source not in self.shm_cams:
            if not self._connect_shm(source):
                return None
        
        shm = self.shm_cams[source]
        try:
            # Read timestamp and data length
            timestamp = int.from_bytes(shm.buf[0:4], 'little')
            data_len = int.from_bytes(shm.buf[4:8], 'little')
            
            if data_len == 0 or data_len > self.max_buffer_size:
                # No data available or invalid data
                return None
            
            # Read image data
            img_bytes = bytes(shm.buf[8:8+data_len])
            
            # Decode JPEG to numpy array
            import cv2
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            if img is not None:
                logger_mp.debug(f"[MultiImageReader] Successfully decoded image from {source}: {img.shape}")
            else:
                logger_mp.warning(f"[MultiImageReader] Failed to decode image from {source}")
            
            return img
            
        except Exception as e:
            logger_mp.error(f"[MultiImageReader] Error reading image from {source}: {e}")
            return None
    
    def read_all_images(self) -> dict:
        """
        Read all available images from all connected sources.
        
        Returns:
            dict: dictionary of source -> image array
        """
        images = {}
        for source in ['head', 'left', 'right']:
            img = self.read_single_image(source)
            if img is not None:
                images[source] = img
        return images
    
    def close(self):
        """Close all shared memory connections."""
        for source, shm in self.shm_cams.items():
            try:
                shm.close()
                logger_mp.info(f"[MultiImageReader] Closed shared memory for {source}")
            except Exception as e:
                logger_mp.error(f"[MultiImageReader] Error closing shared memory for {source}: {e}")
        self.shm_cams.clear()
        logger_mp.info("[MultiImageReader] All shared memory connections closed")
    
    def __del__(self):
        """Destructor - automatically close connections."""
        self.close()
