# Sunray_v2 docs

本目录是 Sunray_v2 二次开发与使用手册的静态站点。

## 使用方式

直接打开：

```bash
xdg-open docs/index.html
```

或用静态服务打开：

```bash
cd docs
python3 -m http.server 8080
```

然后访问 `http://127.0.0.1:8080`。

## 更新内容镜像

`index.html` 在普通 HTTP 服务下会直接读取 `content/*.md`。如果通过 `file://` 双击打开浏览器，浏览器通常禁止 fetch 本地 Markdown，因此需要 `assets/doc-data.js` 做内容镜像。

修改 `content/*.md` 后运行：

```bash
python3 docs/assets/sync-doc-data.py
```

## 文件结构

```text
docs/
├── index.html
├── assets/
│   ├── app.js
│   ├── doc-data.js
│   ├── style.css
│   ├── sync-doc-data.py
│   └── vendor/marked.min.js
└── content/
    ├── chapters.json
    ├── overview/index.md
    ├── common/sunray_msgs.md
    ├── localization/overview.md
    ├── uav_control/sunray_uav_control.md
    ├── ugv_control/sunray_ugv_control.md
    └── ...
```

## 章节结构

`content/` 下每个章节使用一个文件夹。章节顺序、侧边栏分组、小节标题和该章节包含哪些 Markdown 文件，都由 `content/chapters.json` 配置。

新增章节时：

1. 在 `content/` 下新建目录，例如 `my-topic/`。
2. 在目录中创建一个或多个 Markdown 文件，例如 `index.md`、`details.md`。
3. 在 `content/chapters.json` 中增加一项：

```json
{
  "group": "扩展能力",
  "title": "我的章节",
  "files": ["my-topic/index.md", "my-topic/details.md"],
  "sections": [
    {"id": "my-topic", "title": "我的章节"}
  ]
}
```

4. 运行 `python3 docs/assets/sync-doc-data.py`。
