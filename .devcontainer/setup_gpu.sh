#!/bin/bash
# GPU 渲染配置脚本 - 在容器启动时自动执行
# 用于配置 GLVND vendor 文件，使 Gazebo 能够使用 NVIDIA GPU 硬件加速

echo "=== Configuring NVIDIA GPU rendering ==="

# 1. 创建 GLVND EGL vendor 配置目录
mkdir -p /usr/share/glvnd/egl_vendor.d

# 2. 创建 NVIDIA EGL vendor 配置文件
cat > /usr/share/glvnd/egl_vendor.d/10_nvidia.json << 'EOF'
{
    "file_format_version" : "1.0.0",
    "ICD" : {
        "library_path" : "libEGL_nvidia.so.0"
    }
}
EOF

# 3. 创建 Vulkan ICD 配置目录
mkdir -p /usr/share/vulkan/icd.d

# 4. 创建 NVIDIA Vulkan ICD 配置文件
cat > /usr/share/vulkan/icd.d/nvidia_icd.json << 'EOF'
{
    "file_format_version" : "1.0.0",
    "ICD": {
        "library_path": "libGLX_nvidia.so.0",
        "api_version" : "1.3.274"
    }
}
EOF

# 5. 确保 libGLX.so.0 指向 NVIDIA 实现（如果尚未指向）
if [ ! -L /usr/lib/x86_64-linux-gnu/libGLX.so.0 ] || [ "$(readlink /usr/lib/x86_64-linux-gnu/libGLX.so.0)" != "libGLX_nvidia.so.0" ]; then
    echo "Redirecting libGLX.so.0 to NVIDIA implementation..."
    mv /usr/lib/x86_64-linux-gnu/libGLX.so.0 /usr/lib/x86_64-linux-gnu/libGLX.so.0.mesa.bak 2>/dev/null || true
    ln -sf libGLX_nvidia.so.0 /usr/lib/x86_64-linux-gnu/libGLX.so.0
    ldconfig
fi

# 6. 设置环境变量（添加到 bashrc）
if ! grep -q "GAZEBO_GPU_RENDERING" ~/.bashrc 2>/dev/null; then
    echo "" >> ~/.bashrc
    echo "# NVIDIA GPU rendering configuration" >> ~/.bashrc
    echo "export GAZEBO_GPU_RENDERING=1" >> ~/.bashrc
    echo "export __GL_SYNC_TO_VBLANK=0" >> ~/.bashrc
    echo "" >> ~/.bashrc
fi

# 7. 验证配置
echo "=== Verification ==="
if command -v glxinfo &> /dev/null; then
    RENDERER=$(glxinfo 2>/dev/null | grep "OpenGL renderer string" | awk -F': ' '{print $2}')
    if [[ "$RENDERER" == *"llvmpipe"* ]]; then
        echo "⚠️  WARNING: Still using software rendering (llvmpipe)"
        echo "   Please ensure container was started with --gpus all"
    elif [[ "$RENDERER" == *"NVIDIA"* ]] || [[ "$RENDERER" == *"Quadro"* ]] || [[ "$RENDERER" == *"GeForce"* ]] || [[ "$RENDERER" == *"RTX"* ]]; then
        echo "✅ GPU rendering configured successfully!"
        echo "   OpenGL renderer: $RENDERER"
    else
        echo "ℹ️  OpenGL renderer: $RENDERER"
    fi
else
    echo "ℹ️  glxinfo not available, skipping verification"
fi

echo "=== GPU configuration complete ==="
