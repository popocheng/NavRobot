# GPU 渲染配置说明

## 问题描述
在 Docker 容器中运行 Gazebo 时，由于缺少 GLVND vendor 配置文件，Gazebo 会使用 CPU 软件渲染 (llvmpipe)，导致帧率极低 (3-5 FPS)。

## 解决方案

### 自动配置（推荐）
容器启动时会自动执行 `.devcontainer/setup_gpu.sh` 脚本，配置 GPU 渲染。

### 手动配置
如果自动配置未生效，可在容器内手动执行：
```bash
/workspace/.devcontainer/setup_gpu.sh
```

### 验证 GPU 渲染
```bash
glxinfo | grep "OpenGL renderer"
```
- ✅ **正常**: 显示 `Quadro RTX 4000` 或 `NVIDIA` 相关字样
- ❌ **异常**: 显示 `llvmpipe` (软件渲染)

### 启动 Gazebo
```bash
ros2 launch robot_scene go2w_lidar_gps.launch.py
```

## 配置内容
脚本会执行以下操作：
1. 创建 GLVND EGL vendor 配置文件 (`/usr/share/glvnd/egl_vendor.d/10_nvidia.json`)
2. 创建 Vulkan ICD 配置文件 (`/usr/share/vulkan/icd.d/nvidia_icd.json`)
3. 确保 `libGLX.so.0` 指向 NVIDIA 实现
4. 设置环境变量 (`GAZEBO_GPU_RENDERING=1`)
5. 验证配置是否生效

## 注意事项
- 容器必须使用 `--gpus all` 参数启动
- 宿主机必须安装 NVIDIA 驱动 (版本 550.x)
- 需要挂载 X11 socket (`-v /tmp/.X11-unix:/tmp/.X11-unix:rw`)


💡 长期维护建议

    如果未来宿主机驱动升级导致 GPU 渲染失效，只需在容器内重新执行：

     1 /workspace/.devcontainer/setup_gpu.sh

    然后再次 commit 新镜像即可。



## 问题描述
rebuild还是使用之前的镜像:在VSCode 中执行Dev Containers: Rebuild Container，然后docker ps发现新建的容器依然是基于原始镜像的版本（dddmr_gz:20260407_04，应该是dddmr_gz:20260407_05）

### 解决
在宿主机终端执行：
```bash
     1 # 1. 停止并删除当前容器
     2 docker rm -f xenodochial_franklin
     3 
     4 # 2. 清理 VSCode Dev Container 缓存
     5 rm -rf ~/.vscode/extensions/ms-vscode-remote.remote-containers-*/.devcontainer/
     6 
     7 # 3. 在项目目录中清理本地缓存
     8 cd /media/nhy/office/Gazebo/0403/robotdog_nav
     9 rm -rf .devcontainer/.cache 2>/dev/null
```

