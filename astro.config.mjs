// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

// https://astro.build/config
export default defineConfig({
	site: 'https://algo.xuyi.dev',
	integrations: [
		starlight({
			title: '游戏算法图解',
			description: '用可交互的演示和 C++ 代码，讲清楚游戏开发中的常见算法。',
			favicon: '/favicon.svg',
			defaultLocale: 'root',
			locales: {
				root: { label: '简体中文', lang: 'zh-CN' },
			},
			social: [
				{ icon: 'github', label: 'GitHub', href: 'https://github.com/08820048/game_algorithms' },
				{ icon: 'external', label: '博客', href: 'https://xuyi.dev' },
			],
			customCss: [
				'@fontsource/noto-serif-sc/600.css',
				'@fontsource/noto-serif-sc/700.css',
				'@fontsource-variable/jetbrains-mono',
				'./src/styles/custom.css',
			],
			components: {
				PageTitle: './src/components/overrides/PageTitle.astro',
				Footer: './src/components/overrides/Footer.astro',
				SiteTitle: './src/components/overrides/SiteTitle.astro',
			},
			expressiveCode: {
				themes: ['min-dark', 'min-light'],
				styleOverrides: {
					borderRadius: '0',
					borderColor: 'var(--sl-color-hairline-light)',
					codeFontFamily: 'var(--__sl-font-mono)',
					codeFontSize: '0.8125rem',
					codeLineHeight: '1.75',
					codeBackground: 'var(--ga-raised)',
					uiFontFamily: 'var(--__sl-font-mono)',
					uiFontSize: '0.75rem',
					frames: {
						shadowColor: 'transparent',
						editorTabBarBackground: 'var(--ga-paper)',
						editorActiveTabBackground: 'var(--ga-raised)',
						editorActiveTabForeground: 'var(--sl-color-gray-2)',
						editorActiveTabIndicatorTopColor: 'var(--ga-vermilion)',
						editorActiveTabIndicatorBottomColor: 'transparent',
						editorTabBarBorderBottomColor: 'var(--sl-color-hairline-light)',
						terminalBackground: 'var(--ga-raised)',
						terminalTitlebarBackground: 'var(--ga-paper)',
						terminalTitlebarBorderBottomColor: 'var(--sl-color-hairline-light)',
					},
					textMarkers: {
						markBackground: 'color-mix(in srgb, var(--ga-vermilion) 12%, transparent)',
						markBorderColor: 'var(--ga-vermilion)',
					},
				},
			},
			lastUpdated: true,
			sidebar: [
				{ label: '导读', items: [{ autogenerate: { directory: 'guide' } }] },
				{ label: '序 · 基础工具', items: [{ autogenerate: { directory: 'foundations' } }] },
				{ label: 'I · 寻路', items: [{ autogenerate: { directory: 'pathfinding' } }] },
				{ label: 'II · 碰撞检测与物理', items: [{ autogenerate: { directory: 'collision' } }] },
				{ label: 'III · 游戏 AI', items: [{ autogenerate: { directory: 'ai' } }] },
				{ label: 'IV · 程序化生成', items: [{ autogenerate: { directory: 'procgen' } }] },
			],
		}),
	],
});
