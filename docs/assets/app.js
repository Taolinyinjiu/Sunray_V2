(function () {
  const chapters = Array.isArray(window.SUNRAY_CHAPTERS) ? window.SUNRAY_CHAPTERS : [];
  const pageData = Array.isArray(window.SUNRAY_DOCS) ? window.SUNRAY_DOCS : [];
  const pageDataByFile = new Map(pageData.map((page) => [page.file, page]));
  const nav = document.getElementById("doc-nav");
  const content = document.getElementById("doc-content");
  const pageToc = document.getElementById("page-toc");
  const fileToPage = new Map();
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

  function titleFromPath(file) {
    const name = file.split("/").pop().replace(/\.md$/, "");
    if (name === "index") {
      return "概览";
    }
    return name;
  }

  const pages = chapters.flatMap((chapter) => {
    return chapter.files.map((file, index) => {
      const fullFile = `content/${file}`;
      const mirrored = pageDataByFile.get(fullFile);
      const section = Array.isArray(chapter.sections) ? chapter.sections[index] : null;
      const title = chapter.files.length === 1
        ? chapter.title
        : (section && section.title) || (mirrored && mirrored.title) || titleFromPath(file);
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

  function pageHref(page) {
    return `#page=${encodeURIComponent(page.file)}`;
  }

  function headingHref(page, headingId) {
    return `#page=${encodeURIComponent(page.file)}&heading=${encodeURIComponent(headingId)}`;
  }

  function buildNav() {
    if (!nav) {
      return;
    }
    const chunks = [];
    chapters.forEach((chapter) => {
      const chapterPages = chapter.files
        .map((file) => fileToPage.get(`content/${file}`))
        .filter(Boolean);
      if (!chapterPages.length) {
        return;
      }
      const firstPage = chapterPages[0];
      chunks.push(
        `<a class="nav-section" href="${pageHref(firstPage)}" data-page="${firstPage.file}">${chapter.title}</a>`
      );
      if (chapterPages.length > 1) {
        chapterPages.forEach((page) => {
          chunks.push(`<a class="nav-page" href="${pageHref(page)}" data-page="${page.file}">${page.title}</a>`);
        });
      }
    });
    nav.innerHTML = chunks.join("");
  }

  function updateActiveNav(page) {
    document.querySelectorAll(".sidebar nav a").forEach((link) => {
      link.classList.toggle("active", link.getAttribute("data-page") === page.file);
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
