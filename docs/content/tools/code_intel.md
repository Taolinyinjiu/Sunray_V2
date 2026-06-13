<!-- title: code_intel -->

<section id="tools-code-intel">

## code_intel

`tools/code_intel` 当前主要提供编译数据库辅助脚本，用于改善 IDE、clangd、代码索引工具对多 catkin build 目录的识别。

### 主要文件

```text
tools/code_intel/merge_compile_commands.py
```

该脚本用于合并多个 `compile_commands.json`，让编辑器能更准确地跳转、补全和分析 C++ 代码。

### 二次开发

- 如果你只写普通 ROS 节点，通常不需要修改这里。
- 如果新增了复杂构建目录或希望统一生成 clangd 配置，可以在这个目录扩展相关工具。

</section>
