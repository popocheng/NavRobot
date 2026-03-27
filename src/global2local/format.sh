#! /bin/bash
if command -v clang-format &> /dev/null; then
    echo "clang-format is installed. "
    find . -regex '.*\.\(cpp\|hpp\|cc\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
    echo "格式化代码完成"
else
    echo "未找到 clang-format，通过以下命令安装"
    echo "sudo apt install clang-format"
fi

