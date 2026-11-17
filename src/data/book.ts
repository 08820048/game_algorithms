/**
 * 这本书是谁：书名、系列中的位置、主题色。
 * 图解系列的每本书各有一份，组件和样式只从这里取，不写死书名和颜色。
 * 主题色本身在 src/styles/custom.css 的 --book-accent（亮色 / 暗色各一个值）。
 */

export interface SeriesBook {
	id: string;
	name: string; // 书名
	edition: string; // 橙皮书 / 绿皮书 / 蓝皮书
	href?: string; // 还没上线的书不给链接
	color: string; // 系列导航里的色标（亮色主题下的主题色）
}

export const series: SeriesBook[] = [
	{ id: 'algo', name: '图解游戏算法', edition: '橙皮书', href: 'https://algo.xuyi.dev', color: '#c8402a' },
	{ id: 'cpp', name: '图解 C++', edition: '绿皮书', color: '#2e7a55' },
	{ id: 'ds', name: '图解数据结构与算法', edition: '蓝皮书', color: '#2b5c9e' },
];

export const book = {
	id: 'algo',
	name: '图解游戏算法',
	edition: '橙皮书',
	seal: '算',
	seriesName: '图解系列',
};
