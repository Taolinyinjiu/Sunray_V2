<!-- title: build_scripts -->

<section id="tools-build-scripts">

## build_scripts

`tools/build_scripts` 是根目录 `build.sh` 背后的模块化构建脚本。它按功能包和模块组组织编译，方便只构建某一部分工程。

### 关键文件

```text
tools/build_scripts/modules.yaml
tools/build_scripts/bin/
tools/build_scripts/common/
tools/build_scripts/cli/
tools/build_scripts/tui/
```

`modules.yaml` 定义模块名、源码路径、构建路径和依赖关系。根目录 `build.sh --list`、`build.sh --groups` 的内容都来自这里。

### 二次开发

- 新增包后，如果希望被 `build.sh` 管理，需要在 `modules.yaml` 添加模块。
- 如果某个模块依赖 `sunray_msgs` 或 `sunray_common`，应写在 dependencies 中。
- 构建脚本属于项目基础设施，改动前要考虑所有开发者和 CI/部署脚本是否依赖原行为。

</section>
