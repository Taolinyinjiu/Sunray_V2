(function () {
  const chapters = Array.isArray(window.SUNRAY_CHAPTERS) ? window.SUNRAY_CHAPTERS : [];
  const pageData = Array.isArray(window.SUNRAY_DOCS) ? window.SUNRAY_DOCS : [];
  const pageDataByFile = new Map(pageData.map((page) => [page.file, page]));
  const nav = document.getElementById("doc-nav");
  const content = document.getElementById("doc-content");
  const pageToc = document.getElementById("page-toc");
  const fileToPage = new Map();
  const navNodeById = new Map();
  const navNodeByPage = new Map();
  const collapsedStateKey = "sunray-doc-nav-collapsed";
  let currentFile = "";

  function slugify(text) {
    return text
      .trim()
      .toLowerCase()
      .replace(/<[^>]+>/g, "")
      .replace(/[`*_~[\](){}.:/\\,，。；;：！!?？]+/g, "")
      .replace(/\s+/g, "-")
      .replace(/-+/g, "-")
      .replace(/^-|-$/g, "");
  }

  function escapeHtml(text) {
    return String(text)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;")
      .replace(/'/g, "&#39;");
  }

  function titleFromPath(file) {
    const name = file.split("/").pop().replace(/\.md$/, "");
    if (name === "index") {
      return "概览";
    }
    return name;
  }

  function normalizeFile(file) {
    if (!file) {
      return "";
    }
    return file.startsWith("content/") ? file : `content/${file}`;
  }

  function pageHref(page) {
    return `#page=${encodeURIComponent(page.file)}`;
  }

  function headingHref(page, headingId) {
    return `#page=${encodeURIComponent(page.file)}&heading=${encodeURIComponent(headingId)}`;
  }

  function loadCollapsedState() {
    try {
      const raw = localStorage.getItem(collapsedStateKey);
      const parsed = raw ? JSON.parse(raw) : {};
      return parsed && typeof parsed === "object" ? parsed : {};
    } catch (error) {
      return {};
    }
  }

  function saveCollapsedState(state) {
    try {
      localStorage.setItem(collapsedStateKey, JSON.stringify(state));
    } catch (error) {
      // localStorage may be unavailable under strict browser settings.
    }
  }

  const collapsedState = loadCollapsedState();

  function legacyEntries(chapter) {
    const files = Array.isArray(chapter.files) ? chapter.files : [];
    return files.map((file, index) => {
      const fullFile = normalizeFile(file);
      const mirrored = pageDataByFile.get(fullFile);
      const section = Array.isArray(chapter.sections) ? chapter.sections[index] : null;
      const title = files.length === 1
        ? chapter.title
        : (section && section.title) || (mirrored && mirrored.title) || titleFromPath(file);
      return {
        file,
        title,
      };
    });
  }

  function collectTreeEntries(nodes, entries = []) {
    if (!Array.isArray(nodes)) {
      return entries;
    }
    nodes.forEach((node) => {
      if (node.file) {
        entries.push({
          file: node.file,
          title: node.title || titleFromPath(node.file),
        });
      }
      collectTreeEntries(node.children, entries);
    });
    return entries;
  }

  function chapterEntries(chapter) {
    const entries = Array.isArray(chapter.children) && chapter.children.length
      ? collectTreeEntries(chapter.children)
      : legacyEntries(chapter);

    const seen = new Set();
    return entries.filter((entry) => {
      const file = normalizeFile(entry.file);
      if (!file || seen.has(file)) {
        return false;
      }
      seen.add(file);
      return true;
    });
  }

  function chapterDefaultFile(chapter, entries) {
    return normalizeFile(chapter.default || (entries[0] && entries[0].file));
  }

  function buildLegacyChildren(chapter) {
    const entries = legacyEntries(chapter);
    if (entries.length <= 1) {
      return [];
    }
    return entries.map((entry) => ({
      title: entry.title,
      file: entry.file,
    }));
  }

  function buildChapterNode(chapter, index) {
    const entries = chapterEntries(chapter);
    return {
      id: `chapter-${index}-${slugify(chapter.title) || index}`,
      title: chapter.title,
      default: chapterDefaultFile(chapter, entries),
      children: Array.isArray(chapter.children) && chapter.children.length
        ? chapter.children
        : buildLegacyChildren(chapter),
      chapter,
    };
  }

  const chapterNodes = chapters.map(buildChapterNode);

  const pages = chapters.flatMap((chapter) => {
    return chapterEntries(chapter).map((entry) => {
      const fullFile = normalizeFile(entry.file);
      const mirrored = pageDataByFile.get(fullFile);
      const title = entry.title || (mirrored && mirrored.title) || titleFromPath(entry.file);
      const page = {
        file: fullFile,
        hash: `page=${encodeURIComponent(fullFile)}`,
        title,
        chapterTitle: chapter.title,
        group: chapter.group,
      };
      fileToPage.set(fullFile, page);
      return page;
    });
  });

  marked.setOptions({
    gfm: true,
    breaks: false,
    mangle: false,
    headerIds: false,
  });

  function parseHash() {
    const raw = decodeURIComponent(window.location.hash.replace(/^#/, ""));
    const params = new URLSearchParams(raw);
    const file = params.get("page");
    const heading = params.get("heading");
    return {
      file: fileToPage.has(file) ? file : pages[0] && pages[0].file,
      heading,
    };
  }

  function collectNodeFiles(node, files = []) {
    if (node.file) {
      files.push(normalizeFile(node.file));
    }
    if (node.default) {
      files.push(normalizeFile(node.default));
    }
    if (Array.isArray(node.children)) {
      node.children.forEach((child) => collectNodeFiles(child, files));
    }
    return Array.from(new Set(files.filter(Boolean)));
  }

  function nodeTargetPage(node) {
    const file = normalizeFile(node.file || node.default);
    if (fileToPage.has(file)) {
      return fileToPage.get(file);
    }
    const firstFile = collectNodeFiles(node).find((candidate) => fileToPage.has(candidate));
    return firstFile ? fileToPage.get(firstFile) : null;
  }

  function renderNavNode(node, depth, path) {
    const nodeId = path.join("-");
    const children = Array.isArray(node.children) ? node.children : [];
    const hasChildren = children.length > 0;
    const targetPage = nodeTargetPage(node);
    const files = collectNodeFiles(node);
    const collapsed = hasChildren && collapsedState[nodeId] === true;
    const label = escapeHtml(node.title || (targetPage && targetPage.title) || "未命名页面");
    const href = targetPage ? pageHref(targetPage) : "#";
    const dataPage = targetPage ? ` data-page="${escapeHtml(targetPage.file)}"` : "";
    const dataFiles = files.length ? ` data-files="${escapeHtml(files.join("|"))}"` : "";
    const kind = depth === 0 ? "nav-section" : (hasChildren ? "nav-group" : "nav-page");
    const collapsedClass = collapsed ? " is-collapsed" : "";
    const toggle = hasChildren
      ? `<button class="nav-toggle" type="button" aria-label="展开或折叠 ${label}" aria-expanded="${!collapsed}" data-toggle="${nodeId}"></button>`
      : '<span class="nav-toggle-placeholder"></span>';

    navNodeById.set(nodeId, {
      id: nodeId,
      files,
      parentIds: path.slice(0, -1).map((_, index) => path.slice(0, index + 1).join("-")),
    });
    files.forEach((file) => {
      const list = navNodeByPage.get(file) || [];
      list.push(nodeId);
      navNodeByPage.set(file, list);
    });

    const item = [
      `<div class="nav-item ${hasChildren ? "nav-branch" : "nav-leaf"} depth-${depth}${collapsedClass}" data-node="${nodeId}"${dataFiles}>`,
      `<a class="${kind} nav-link" href="${href}"${dataPage}>${label}</a>`,
      toggle,
      "</div>",
    ].join("");

    if (!hasChildren) {
      return item;
    }

    const childHtml = children
      .map((child, index) => renderNavNode(child, depth + 1, path.concat(index)))
      .join("");
    return [
      item,
      `<div class="nav-children depth-${depth + 1}${collapsedClass}" data-parent="${nodeId}">`,
      childHtml,
      "</div>",
    ].join("");
  }

  function buildNav() {
    if (!nav) {
      return;
    }
    navNodeById.clear();
    navNodeByPage.clear();
    nav.innerHTML = chapterNodes
      .map((node, index) => renderNavNode(node, 0, [index]))
      .join("");

    nav.querySelectorAll("[data-toggle]").forEach((button) => {
      button.addEventListener("click", () => {
        const nodeId = button.getAttribute("data-toggle");
        const isCollapsed = collapsedState[nodeId] === true;
        collapsedState[nodeId] = !isCollapsed;
        saveCollapsedState(collapsedState);
        setNodeCollapsed(nodeId, !isCollapsed);
      });
    });
  }

  function setNodeCollapsed(nodeId, collapsed) {
    const item = nav.querySelector(`[data-node="${nodeId}"]`);
    const children = nav.querySelector(`[data-parent="${nodeId}"]`);
    const toggle = nav.querySelector(`[data-toggle="${nodeId}"]`);
    if (item) {
      item.classList.toggle("is-collapsed", collapsed);
    }
    if (children) {
      children.classList.toggle("is-collapsed", collapsed);
    }
    if (toggle) {
      toggle.setAttribute("aria-expanded", String(!collapsed));
    }
  }

  function expandNode(nodeId) {
    if (!nodeId) {
      return;
    }
    collapsedState[nodeId] = false;
    setNodeCollapsed(nodeId, false);
  }

  function expandActivePath(page) {
    const nodeIds = navNodeByPage.get(page.file) || [];
    nodeIds.forEach((nodeId) => {
      const nodeMeta = navNodeById.get(nodeId);
      if (!nodeMeta) {
        return;
      }
      nodeMeta.parentIds.forEach(expandNode);
    });
  }

  function updateActiveNav(page) {
    expandActivePath(page);
    document.querySelectorAll(".sidebar nav [data-page]").forEach((link) => {
      link.classList.toggle("active", link.getAttribute("data-page") === page.file);
    });
    document.querySelectorAll(".sidebar nav [data-files]").forEach((node) => {
      const files = (node.getAttribute("data-files") || "").split("|");
      node.classList.toggle("active-parent", files.includes(page.file));
    });
  }

  function setLoading(page) {
    content.innerHTML = [
      '<div class="loading-panel">',
      "<h2>正在加载文档</h2>",
      `<p>正在读取 ${page ? page.title : "Markdown 内容"}...</p>`,
      "</div>",
    ].join("");
    if (pageToc) {
      pageToc.innerHTML = "";
    }
  }

  function setError(page, error) {
    content.innerHTML = [
      '<section class="load-error">',
      "<h2>文档加载失败</h2>",
      `<p>无法读取 <code>${page.file}</code>。</p>`,
      "<p>如果你正在双击打开 HTML，请确认 <code>assets/doc-data.js</code> 存在；如果通过静态服务打开，请确认 <code>content/</code> 下的 Markdown 文件路径没有改变。</p>",
      `<pre><code>${String(error && error.message ? error.message : error)}</code></pre>`,
      "</section>",
    ].join("");
    if (pageToc) {
      pageToc.innerHTML = "";
    }
  }

  async function loadPageMarkdown(page) {
    if (window.location.protocol === "file:") {
      const mirrored = pageDataByFile.get(page.file);
      if (!mirrored) {
        throw new Error("file:// 模式缺少 doc-data.js 中的内容镜像");
      }
      return mirrored.content;
    }

    try {
      const response = await fetch(page.file, { cache: "no-cache" });
      if (!response.ok) {
        throw new Error(`${response.status} ${response.statusText}`);
      }
      return await response.text();
    } catch (error) {
      const mirrored = pageDataByFile.get(page.file);
      if (mirrored) {
        return mirrored.content;
      }
      throw error;
    }
  }

  function assignHeadingIds(page) {
    const used = new Map();
    content.querySelectorAll("h2, h3, h4").forEach((heading) => {
      if (heading.id) {
        return;
      }
      const base = slugify(heading.textContent) || "heading";
      const count = used.get(base) || 0;
      used.set(base, count + 1);
      heading.id = count ? `${base}-${count + 1}` : base;
    });

    content.querySelectorAll("section[id]").forEach((section) => {
      const firstHeading = section.querySelector("h2, h3, h4");
      if (firstHeading && !firstHeading.id) {
        firstHeading.id = section.id;
      }
    });
  }

  function buildPageToc(page) {
    if (!pageToc) {
      return;
    }
    const headings = Array.from(content.querySelectorAll("h2, h3, h4")).filter((heading) => heading.id);
    if (!headings.length) {
      pageToc.innerHTML = '<p class="toc-empty">暂无目录</p>';
      return;
    }
    pageToc.innerHTML = headings
      .map((heading) => {
        const level = heading.tagName.toLowerCase();
        return `<a class="toc-link toc-${level}" href="${headingHref(page, heading.id)}" data-heading="${heading.id}">${heading.textContent}</a>`;
      })
      .join("");
  }

  function updateActiveToc(headingId) {
    if (!pageToc) {
      return;
    }
    pageToc.querySelectorAll("a").forEach((link) => {
      link.classList.toggle("active", link.getAttribute("data-heading") === headingId);
    });
  }

  function scrollToHeading(headingId) {
    const target = headingId ? document.getElementById(headingId) : null;
    if (!target) {
      window.scrollTo({ top: 0, behavior: "smooth" });
      updateActiveToc("");
      return;
    }
    requestAnimationFrame(() => {
      target.scrollIntoView({ behavior: "smooth", block: "start" });
      updateActiveToc(headingId);
    });
  }

  async function route() {
    if (!pages.length) {
      return;
    }
    const state = parseHash();
    const page = fileToPage.get(state.file) || pages[0];
    updateActiveNav(page);

    try {
      if (currentFile !== page.file) {
        setLoading(page);
        const markdown = await loadPageMarkdown(page);
        content.innerHTML = marked.parse(markdown);
        assignHeadingIds(page);
        buildPageToc(page);
        currentFile = page.file;
      }
      scrollToHeading(state.heading);
    } catch (error) {
      setError(page, error);
    }
  }

  window.addEventListener("hashchange", route);
  document.addEventListener("DOMContentLoaded", () => {
    buildNav();
    route();
  });
})();
