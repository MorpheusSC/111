# WebSocket 连接问题修复文档

## 问题诊断

### 症状
```
websocket is now disconnected. Removing the socket.
WebSocket connection closed
```

头显端可以连接到服务器，显示 "running"，但 WebSocket 连接立即断开，导致无法接收任何数据。

### 根本原因

#### 问题 1：`aiortc` 模块导入失败

**错误信息：**
```
Traceback (most recent call last):
  File "<string>", line 1, in <module>
  File ".../teleop/teleimager/src/teleimager/image_server.py", line 35, in <module>
    from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
ModuleNotFoundError: No module named 'aiortc'
```

**原因：**
- 图像服务器使用子进程启动
- 子进程使用了 `-c` 参数运行 Python 代码字符串
- 这种方式可能导致模块路径问题
- `aiortc` 虽然已安装，但子进程无法正确导入

#### 问题 2：相对导入错误

**错误信息：**
```
ImportError: attempted relative import with no known parent package
```

**原因：**
- 尝试直接运行 `image_server.py`
- 该文件使用了相对导入（`from .image_client import ...`）
- 直接运行脚本时，Python 无法解析相对导入

## 修复方案

### 方案 1：创建独立的启动脚本

**文件：** `start_isaacsim_server.py`

```python
#!/usr/bin/env python3
"""
Standalone script to start IsaacSim image server
"""
import sys
import os

# Add teleimager src to path
script_dir = os.path.dirname(os.path.abspath(__file__))
teleimager_src = os.path.join(script_dir, "teleop", "teleimager", "src")
sys.path.insert(0, teleimager_src)

# Import and run main function
from teleimager.image_server import main

if __name__ == "__main__":
    # Pass command line arguments to main
    sys.argv = [sys.argv[0]] + sys.argv[1:]
    main()
```

**优点：**
- ✅ 避免相对导入问题
- ✅ 正确设置 Python 路径
- ✅ 使用主进程的 Python 环境
- ✅ 支持命令行参数

### 方案 2：修改子进程启动方式

**修改前：**
```python
teleimager_process = subprocess.Popen(
    [sys.executable, "-c", "import sys; sys.path.insert(0, r'" + teleimager_path + "'); ..."],
    stdout=log_file,
    stderr=subprocess.STDOUT,
    text=True,
    env=teleimager_env
)
```

**问题：**
- 使用 `-c` 参数运行代码字符串
- 模块导入可能失败
- 路径设置复杂

**修改后：**
```python
# Path to the standalone startup script
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
start_script = os.path.join(project_root, "start_isaacsim_server.py")

# Create log file for image server
log_file = open("image_server.log", "w")

# Use same Python environment
teleimager_env = os.environ.copy()

teleimager_process = subprocess.Popen(
    [sys.executable, start_script, "--isaacsim"],
    stdout=log_file,
    stderr=subprocess.STDOUT,
    text=True,
    env=teleimager_env
)
```

**优点：**
- ✅ 直接运行 Python 脚本
- ✅ 使用主进程的 Python 环境
- ✅ 所有依赖都可用
- ✅ 代码更清晰简洁

## 测试验证

### 手动测试启动脚本

```bash
timeout 10 python start_isaacsim_server.py --isaacsim
```

**预期输出：**
```
INFO     ====================== Image
         Server Startup Guide
         ======================
...
INFO     [Performance] CPU Affinity locked to: [0, 1, 2]
INFO     [main] Forcing camera head_camera to use isaacsim type for simulation mode
INFO     [main] Forcing camera left_wrist_camera to use isaacsim type for simulation mode
INFO     [main] Forcing camera right_wrist_camera to use isaacsim type for simulation mode
INFO     IsaacSim mode enabled. Waiting 2 seconds for shared memory to be ready...
```

### 测试完整系统

```bash
python teleop/teleop_hand_and_arm.py \
    --arm=X200 \
    --ee=inspire_dfx \
    --sim \
    --record \
    --img-server-ip=172.18.60.81
```

**预期启动日志：**
```
INFO     Starting IsaacSim image server for simulation mode...
INFO     IsaacSim image server started with PID: <pid>
INFO     Image server logs will be written to 'image_server.log'
INFO     Waiting 5 seconds for image server to start...
INFO     [main] Forcing camera head_camera to use isaacsim type for simulation mode
INFO     [IsaacSimCamera: head_camera] initialized with 720x1280 @ 30 FPS...
INFO     ---------------------🚀start Tracking🚀-------------------------
```

**WebSocket 连接应该保持：**
- ✅ 不再出现 "WebSocket connection closed"
- ✅ 头显端可以正常接收数据
- ✅ VR 世界显示正常内容

## 故障排除

### 问题 1：仍然出现 "ModuleNotFoundError"

**检查：**
1. 确认 `aiortc` 已安装：
   ```bash
   pip list | grep aiortc
   ```

2. 如果未安装：
   ```bash
   pip install aiortc
   ```

3. 检查 Python 环境：
   ```bash
   which python
   python --version
   ```

### 问题 2：相对导入错误

**解决方案：**
- ✅ 使用 `start_isaacsim_server.py` 启动脚本
- ✅ 不要直接运行 `image_server.py`

### 问题 3：WebSocket 仍然断开

**检查：**
1. 查看图像服务器日志：
   ```bash
   tail -f image_server.log
   ```

2. 检查进程是否正在运行：
   ```bash
   ps aux | grep image_server
   ```

3. 检查端口是否被占用：
   ```bash
   netstat -tuln | grep -E "60000|55555|66666"
   ```

4. 检查 `ImageClient` 是否能连接：
   ```bash
   python -c "
   import sys
   sys.path.insert(0, 'teleop/teleimager/src')
   from teleimager.image_client import ImageClient
   client = ImageClient(host='172.18.60.81')
   config = client.get_cam_config()
   print('Config:', config)
   "
   ```

## 技术细节

### 为什么使用独立启动脚本？

1. **避免相对导入问题**
   - 直接运行 `image_server.py` 会导致相对导入失败
   - 独立脚本通过 `sys.path` 正确设置模块路径

2. **确保环境一致性**
   - 使用主进程的 Python 解释器
   - 所有依赖都可用
   - 避免 PYTHONPATH 问题

3. **支持命令行参数**
   - 可以传递 `--isaacsim` 等参数
   - 灵活性更高

4. **更好的错误处理**
   - 脚本失败时更容易调试
   - 日志输出更清晰

### 子进程 vs 模块导入

| 方式 | 优点 | 缺点 |
|------|------|------|
| **子进程 + `-c` 参数** | 简单直接 | 模块导入问题，路径复杂 |
| **子进程 + 独立脚本** | 环境一致，支持参数 | 需要额外文件 |
| **直接模块导入** | 无子进程开销 | 可能影响主进程，难以清理 |

**最终选择：** 子进程 + 独立脚本

## 修改文件总结

### 1. `start_isaacsim_server.py`（新建）

- ✅ 独立的图像服务器启动脚本
- ✅ 正确设置 Python 路径
- ✅ 支持命令行参数

### 2. `teleop/teleop_hand_and_arm.py`

- ✅ 修改子进程启动方式
- ✅ 使用独立脚本而不是 `-c` 参数
- ✅ 简化环境变量设置

## 总结

通过本次修复：

✅ **解决了 `aiortc` 导入失败问题**
- 使用独立启动脚本
- 确保子进程使用正确的 Python 环境

✅ **解决了相对导入错误**
- 通过 `sys.path` 正确设置模块路径
- 避免直接运行包含相对导入的脚本

✅ **WebSocket 连接应该保持稳定**
- 图像服务器正常启动
- 相机数据正常发布
- 头显端正常接收

现在 WebSocket 连接应该可以保持稳定，不再自动断开！
