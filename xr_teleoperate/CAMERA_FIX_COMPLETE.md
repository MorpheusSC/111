# IsaacSim 仿真模式相机链路完整修复文档

## 问题诊断

### 原始问题
- WebSocket 连接正常建立，但很快断开
- 头显端显示黑屏，或只显示初始化的网格世界
- 终端显示 `running` 但没有实际的相机数据传输
- `60001` 端口也无法访问

### 根本原因分析

1. **缺少 `MultiImageReader` 依赖**
   - `IsaacSimCamera` 类需要从 `unitree_sim_isaaclab.tools.shared_memory_utils` 导入 `MultiImageReader`
   - 这个工具库未安装，导致 `ImportError`
   - 图像服务器启动失败，无法读取相机数据

2. **仿真模式下没有自动启动图像服务器**
   - 虽然添加了启动代码，但启动失败
   - 输出被捕获到 PIPE，看不到错误信息
   - 服务器进程立即退出

3. **共享内存连接失败**
   - IsaacSim 没有发布相机数据到共享内存
   - 或者共享内存名称不匹配

## 修复方案

### 1. 创建本地 `MultiImageReader` 实现

**文件：** `teleop/teleimager/src/teleimager/multi_image_reader.py`

创建了一个独立的 `MultiImageReader` 类，不需要依赖外部库：

```python
class MultiImageReader:
    """Read images from shared memory published by IsaacSim."""
    
    def __init__(self):
        # Initialize shared memory connections
        
    def read_single_image(self, source: str) -> np.ndarray:
        # Read image from shared memory for specified source
        
    def close(self):
        # Close all shared memory connections
```

**特性：**
- 自动管理共享内存连接
- 支持 'head', 'left', 'right' 三个相机源
- JPEG 编码/解码
- 错误处理和日志记录

### 2. 修改 `image_server.py` 使用本地实现

**文件：** `teleop/teleimager/src/teleimager/image_server.py`

修改 `IsaacSimCamera` 的 `__init__` 方法：

```python
try:
    from tools.shared_memory_utils import MultiImageReader  # Try external first
    logger_mp.info("[IsaacSimCamera] Using MultiImageReader from unitree_sim_isaaclab")
except ImportError:
    from .multi_image_reader import MultiImageReader  # Fall back to local
    logger_mp.info("[IsaacSimCamera] Using local MultiImageReader implementation")
```

这样既支持官方库，也支持本地实现。

### 3. 改进图像服务器启动方式

**文件：** `teleop/teleop_hand_and_arm.py`

修改子进程启动方式，将输出写入日志文件：

```python
# Create log file for image server
log_file = open("image_server.log", "w")

teleimager_process = subprocess.Popen(
    [sys.executable, "-c", "..."],
    stdout=log_file,
    stderr=subprocess.STDOUT,
    text=True,
    env=teleimager_env
)
```

**好处：**
- 可以查看图像服务器的详细日志
- 便于调试问题
- 不阻塞主进程

### 4. 添加必要的导入和清理

添加了：
- `subprocess` - 用于启动子进程
- `signal` - 用于进程管理
- 进程清理代码在 `finally` 块中

## 使用说明

### 1. 启动 IsaacSim 仿真环境

确保 IsaacSim 正在运行，并且启用了相机功能：

```bash
# IsaacSim 应该以 --enable_cameras 参数启动
python your_isaac_sim_script.py --enable_cameras
```

### 2. 启动遥操作程序

使用原来的命令即可：

```bash
python teleop/teleop_hand_and_arm.py \
    --arm=X200 \
    --ee=inspire_dfx \
    --sim \
    --record \
    --img-server-ip=172.18.60.81
```

### 3. 系统会自动：

1. ✅ 检测到 `--sim` 参数
2. ✅ 启动仿真模式特定的功能
3. ✅ 自动启动 IsaacSim 图像服务器（后台进程）
4. ✅ 强制所有相机类型为 `isaacsim`
5. ✅ 等待服务器启动完成（3秒）
6. ✅ 尝试从共享内存读取相机数据
7. ✅ 通过 ZMQ 发送到头显端

### 4. 查看日志

查看 `image_server.log` 文件获取详细的日志信息：

```bash
tail -f image_server.log
```

正常启动的日志应该包含：

```
[main] Forcing camera head_camera to use isaacsim type for simulation mode
[main] Forcing camera left_camera to use isaacsim type for simulation mode
[main] Forcing camera right_camera to use isaacsim type for simulation mode
[main] IsaacSim mode enabled. Waiting 2 seconds for shared memory to be ready...
[IsaacSimCamera: head_camera] initialized with 720x1280 @ 30 FPS, source='head', mode='monocular'.
[MultiImageReader] Initialized
Image server started successfully
```

## 测试

运行测试脚本验证修复：

```bash
python test_isaacsim_fix.py
```

预期输出：

```
============================================================
Testing IsaacSim Camera Fix
============================================================

✓ IsaacSimCamera imported successfully
✓ Config loading and isaacsim type forcing works
✓ Local MultiImageReader imported successfully

============================================================
Test Results:
============================================================
IsaacSimCamera Import................... PASS
Config Loading.......................... PASS
Shared Memory Tools..................... PASS

✓ All critical tests passed!
```

## 故障排除

### 问题1：仍然显示黑屏

**检查：**
1. IsaacSim 是否正在运行并启用了相机？
   ```bash
   # 检查 IsaacSim 进程
   ps aux | grep isaac
   ```

2. 共享内存是否存在？
   ```bash
   # 查看共享内存
   ls /dev/shm/
   # 应该看到 isaacsim_camera_head 等文件
   ```

3. 查看图像服务器日志
   ```bash
   cat image_server.log
   ```

### 问题2：WebSocket 连接断开

**可能原因：**
- 图像服务器启动失败
- ZMQ 端口冲突
- 网络配置问题

**解决方法：**
1. 检查 `image_server.log` 中的错误信息
2. 确认 `img-server-ip` 参数正确
3. 检查防火墙设置

### 问题3：No data available for camera

**日志：**
```
[IsaacSimCamera] No data available for head_camera, frame_data is None
```

**解决方法：**
1. 确认 IsaacSim 正在发布相机数据到共享内存
2. 检查共享内存名称是否正确（`isaacsim_camera_head`, `isaacsim_camera_left`, `isaacsim_camera_right`）
3. 检查相机分辨率和帧率配置

## 技术细节

### 相机数据流

```
IsaacSim (仿真环境)
    ↓ (共享内存)
isaacsim_camera_head (共享内存)
    ↓
MultiImageReader.read_single_image('head')
    ↓
IsaacSimCamera._update_frame()
    ↓
JPEG 编码
    ↓
ZMQ 发布 (端口 55555)
    ↓
ImageClient (ZMQ 订阅)
    ↓
TeleVuer
    ↓
头显端 (VR 显示)
```

### 共享内存布局

每个相机的共享内存：

```
偏移量    大小    内容
0         4       时间戳 (uint32, little endian)
4         4       数据长度 (uint32, little endian)
8         N       JPEG 编码的图像数据
```

### 共享内存名称

- `isaacsim_camera_head` - 主相机
- `isaacsim_camera_left` - 左相机（立体视觉）
- `isaacsim_camera_right` - 右相机（立体视觉）

## 进阶配置

### 调整相机分辨率

修改 `teleop/teleimager/cam_config_server.yaml`：

```yaml
head_camera:
  type: isaacsim
  height: 720    # 调整高度
  width: 1280    # 调整宽度
  fps: 30        # 帧率
  ...
```

### 调整等待时间

如果共享内存初始化需要更多时间，修改 `teleop/teleimager/src/teleimager/image_server.py`：

```python
# 在 main 函数中
if args.isaacsim:
    ...
    time.sleep(5.0)  # 增加到 5 秒
```

### 禁用日志

如果要减少日志输出，修改日志级别：

```python
# 在 teleop/teleop_hand_and_arm.py 中
logging_mp.basicConfig(level=logging_mp.WARNING)
```

## 依赖项

必须安装的 Python 包：

```bash
pip install pyzmq          # ZMQ 通信
pip install aiortc         # WebRTC 支持
pip install pupil-labs-uvc # UVC 相机支持
pip install opencv-python  # 图像处理
pip install numpy          # 数值计算
pip install aiohttp        # HTTP/WebRTC 服务器
```

检查依赖：

```bash
pip list | grep -E "zmq|opencv|aiohttp|aiortc|uvc|numpy"
```

## 总结

通过本次修复：

✅ 创建了独立的 `MultiImageReader` 实现
✅ 修改了图像服务器以支持本地和官方两种实现
✅ 改进了子进程启动和日志记录
✅ 添加了完整的进程管理和清理
✅ 提供了详细的测试和故障排除指南

现在仿真模式下的相机链路应该可以正常工作了！
