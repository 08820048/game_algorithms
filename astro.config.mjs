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
			defaultLocale: 'root',
			locales: {
				root: { label: '简体中文', lang: 'zh-CN' },
			},
			social: [
				{ icon: 'github', label: 'GitHub', href: 'https://github.com/08820048/game_algorithms' },
				{ icon: 'external', label: '博客', href: 'https://xuyi.dev' },
			],
			customCss: ['./src/styles/custom.css'],
			lastUpdated: true,
			sidebar: [
				{ label: '开始', items: [{ autogenerate: { directory: 'guide' } }] },
				{ label: '模块一 · 寻路', items: [{ autogenerate: { directory: 'pathfinding' } }] },
			],
		}),
	],
});
