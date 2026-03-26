# 启动顺序和 finally 块修复文档

## 问题诊断

### 原始错误信息

```
10:04:45.885163 ERROR    Failed to ctrl_dual_arm_go_home: name 'arm_ctrl' is not defined
10:04:46.065046 INFO     Image client has been closed.
10:04:46.069581 ERROR    Failed to stop sim state subscriber: name 'sim_state_subscriber' is not defined
10:04:46.069745 ERROR    Failed to close recorder: name 'recorder' is not defined
10:04:46.069745 INFO     ✅ Finally, exiting program.
```

### 问题根源

#### 问题 1：`finally` 块中的变量未定义

在 `finally` 块中使用了 `arm_ctrl`、`sim_state_subscriber` 和 `recorder` 等变量，但这些变量是在 `try` 块内部初始化的。如果初始化过程中出现错误，这些变量就不会被定义，导致 `finally` 块在尝试使用它们时报错。

**错误代码：**
```python
finally:
    try:
        arm_ctrl.ctrl_dual_arm_go_home()  # arm_ctrl 可能未定义
    except Exception as e:
        logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")
```

**修复后：**
```python
finally:
    try:
        if 'arm_ctrl' in locals():
            arm_ctrl.ctrl_dual_arm_go_home()
    except Exception as e:
        logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")
```

#### 问题 2：启动顺序错误 - 导致程序启动失败

更严重的问题是启动顺序错误：

1. **第 123 行**：初始化 `ImageClient`，尝试连接图像服务器（端口 60000）
2. **第 249 行**：启动 IsaacSim 图像服务器

**错误流程：**
```
初始化 ImageClient
  → 尝试连接图像服务器（端口 60000）
  → 连接失败（服务器还没启动）
  → RuntimeError: "Failed to get camera configuration."
  → 程序退出
  → 进入 finally 块
  → 变量未定义错误
```

## 修复方案

### 1. 修复 `finally` 块变量检查

对所有可能在 `try` 块中未定义的变量添加 `'variable' in locals()` 检查：

```python
finally:
    try:
        if 'arm_ctrl' in locals():
            arm_ctrl.ctrl_dual_arm_go_home()
    except Exception as e:
        logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")

    try:
        if args.ipc and 'ipc_server' in locals():
            ipc_server.stop()
        elif 'listen_keyboard_thread' in locals():
            stop_listening()
            listen_keyboard_thread.join()
    except Exception as e:
        logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")

    try:
        if 'img_client' in locals():
            img_client.close()
    except Exception as e:
        logger_mp.error(f"Failed to close image client: {e}")

    try:
        if 'tv_wrapper' in locals():
            tv_wrapper.close()
    except Exception as e:
        logger_mp.error(f"Failed to close televuer wrapper: {e}")

    try:
        if args.sim and 'sim_state_subscriber' in locals():
            sim_state_subscriber.stop_subscribe()
    except Exception as e:
        logger_mp.error(f"Failed to stop sim state subscriber: {e}")

    try:
        if args.record and 'recorder' in locals():
            recorder.close()
    except Exception as e:
        logger_mp.error(f"Failed to close recorder: {e}")
```

### 2. 修复启动顺序

将仿真模式初始化（包括图像服务器启动）移到 `ImageClient` 初始化之前：

**修改后的顺序：**

```
1. 初始化 DDS 通信
2. 初始化 IPC/键盘监听
3. 【仿真模式检查】
   a. 启动 IsaacSim 图像服务器
   b. 等待 5 秒让服务器完全启动
   c. 检查进程是否仍在运行
4. 初始化 ImageClient（现在服务器已经就绪）
5. 初始化 TeleVuerWrapper
6. 初始化手臂控制器
7. 初始化末端执行器
8. 初始化录制器
9. 开始主循环
```

**关键代码：**

```python
# 仿真模式 - 在 ImageClient 之前启动
if args.sim:
    # 启动仿真状态订阅
    sim_state_subscriber = start_sim_state_subscribe()

    # 启动 IsaacSim 图像服务器
    logger_mp.info("Starting IsaacSim image server for simulation mode...")
    teleimager_process = subprocess.Popen([...])
    logger_mp.info(f"IsaacSim image server started with PID: {teleimager_process.pid}")

    # 等待 5 秒让服务器完全启动
    logger_mp.info("Waiting 5 seconds for image server to start...")
    time.sleep(5.0)

    # 检查进程是否仍在运行
    if teleimager_process.poll() is not None:
        logger_mp.error("IsaacSim image server exited early!")
        raise RuntimeError("Image server failed to start")

# 现在可以安全地初始化 ImageClient
img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
camera_config = img_client.get_cam_config()
```

### 3. 增强错误检查

在启动图像服务器后，检查进程是否仍在运行：

```python
# 检查进程是否仍在运行
if teleimager_process.poll() is not None:
    logger_mp.error("IsaacSim image server exited early! Check 'image_server.log' for errors.")
    raise RuntimeError("Image server failed to start")
```

这样可以：
- 及早发现问题
- 提供清晰的错误信息
- 防止后续代码在服务器未就绪时执行

## 测试验证

运行测试脚本验证修复：

```bash
python test_startup_fix.py
```

预期输出：

```
✓ IsaacSimCamera imported successfully
✓ MultiImageReader imported successfully
✓ Simulation mode code comes BEFORE ImageClient initialization
✓ finally block checks for 'arm_ctrl' before use
✓ finally block checks for 'sim_state_subscriber' before use
✓ finally block checks for 'recorder' before use

✓ All checks passed!

The startup order fix is correct:
1. Simulation mode detected
2. IsaacSim image server started
3. Wait 5 seconds for server to be ready
4. Initialize ImageClient (now server is ready)
5. Continue with rest of initialization
```

## 修改文件总结

### 1. `teleop/teleop_hand_and_arm.py`

**修改内容：**
- ✅ 将仿真模式初始化移到 `ImageClient` 初始化之前
- ✅ 增加等待时间到 5 秒
- ✅ 添加进程状态检查
- ✅ 在 `finally` 块中添加变量存在性检查
- ✅ 删除重复的仿真模式代码

**关键变更：**
- 第 123-167 行：仿真模式初始化（包括图像服务器启动）
- 第 169 行：`ImageClient` 初始化
- 第 533-588 行：`finally` 块，添加了 `in locals()` 检查

### 2. `teleop/teleimager/src/teleimager/multi_image_reader.py`

**新建文件：**
- ✅ 创建本地 `MultiImageReader` 实现
- ✅ 支持从共享内存读取 IsaacSim 相机数据
- ✅ 自动管理共享内存连接

### 3. `teleop/teleimager/src/teleimager/image_server.py`

**修改内容：**
- ✅ 修改 `IsaacSimCamera.__init__()` 支持本地和官方两种实现
- ✅ 添加 `--isaacsim` 命令行参数
- ✅ 自动强制相机类型为 `isaacsim`

## 使用说明

### 正确的启动流程

1. **启动 IsaacSim 仿真环境**
   ```bash
   # 确保启用了相机
   python your_isaac_sim_script.py --enable_cameras
   ```

2. **启动遥操作程序**
   ```bash
   python teleop/teleop_hand_and_arm.py \
       --arm=X200 \
       --ee=inspire_dfx \
       --sim \
       --record \
       --img-server-ip=172.18.60.81
   ```

3. **观察启动日志**

   正常启动应该看到：

   ```
   INFO  Starting IsaacSim image server for simulation mode...
   INFO  IsaacSim image server started with PID: <pid>
   INFO  Image server logs will be written to 'image_server.log'
   INFO  Waiting 5 seconds for image server to start...
   INFO  [main] Forcing camera head_camera to use isaacsim type for simulation mode
   INFO  [main] Forcing camera left_camera to use isaacsim type for simulation mode
   INFO  [main] Forcing camera right_camera to use isaacsim type for simulation mode
   INFO  [main] IsaacSim mode enabled. Waiting 2 seconds for shared memory to be ready...
   INFO  [IsaacSimCamera: head_camera] initialized with 720x1280 @ 30 FPS...
   INFO  ---------------------🚀start Tracking🚀-------------------------
   ```

4. **检查图像服务器日志**
   ```bash
   tail -f image_server.log
   ```

## 故障排除

### 问题 1：仍然显示黑屏

**检查：**
1. IsaacSim 是否正在运行并启用了相机？
   ```bash
   ps aux | grep isaac
   ```

2. 共享内存是否存在？
   ```bash
   ls /dev/shm/ | grep isaacsim_camera
   ```

3. 图像服务器是否正在运行？
   ```bash
   ps aux | grep image_server
   ```

4. 查看图像服务器日志：
   ```bash
   cat image_server.log
   ```

### 问题 2：图像服务器启动失败

**可能原因：**
- 缺少依赖（pyzmq, aiortc 等）
- 端口被占用
- 共享内存权限问题

**解决方法：**
1. 检查依赖：
   ```bash
   pip list | grep -E "zmq|opencv|aiohttp|aiortc"
   ```

2. 检查端口：
   ```bash
   netstat -tuln | grep -E "60000|55555"
   ```

3. 查看日志文件：
   ```bash
   cat image_server.log
   ```

### 问题 3：程序启动后立即退出

**检查：**
1. 是否有运行时错误？
2. 查看完整的终端输出
3. 检查 `image_server.log` 是否有错误信息

**常见错误：**
- `RuntimeError: Failed to get camera configuration.` - 图像服务器未启动或连接失败
- `NameError: name 'xxx' is not defined` - 变量未定义（应该已经修复）
- `ImportError` - 缺少依赖

## 总结

通过本次修复：

✅ **解决了启动顺序问题**
- 图像服务器在 ImageClient 之前启动
- 给予足够的时间让服务器完全启动
- 添加了进程状态检查

✅ **解决了 finally 块变量未定义问题**
- 所有变量使用前都检查 `in locals()`
- 避免程序异常退出时的 NameError

✅ **创建了本地 MultiImageReader**
- 不依赖外部库
- 支持从共享内存读取相机数据
- 自动管理连接和错误处理

✅ **提供了完整的测试和文档**
- 测试脚本验证修复
- 详细的故障排除指南
- 清晰的使用说明

现在程序应该可以正常启动并运行了！
