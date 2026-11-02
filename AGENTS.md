## 项目说明

游戏算法教程站：Astro Starlight 文章 + C++/raylib 演示（Emscripten 编译为 wasm，iframe 嵌入）。详见 README.md。

- 修改 `cpp/` 后：`npm run cpp:test` 跑测试，`npm run demos:build` 重新生成 `public/demos/`
- 文章代码通过 `<CppSnippet file region>` 从 `cpp/` 引用，region 用 `// #region name` / `// #endregion` 标记
- 演示画布内文字用英文；网页 UI 与文章用中文

## Development

When starting the dev server, use background mode:

```
astro dev --background
```

Manage the background server with `astro dev stop`, `astro dev status`, and `astro dev logs`.

Full documentation: https://docs.astro.build · https://starlight.astro.build
