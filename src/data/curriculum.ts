// 全书目录：首页目录、路线图、文章页眉都从这里读取。
// 发布一篇新文章时：把对应章节的 status 改为 'published' 并填上 href。

export type ChapterStatus = 'published' | 'writing' | 'planned';

export interface Chapter {
	no: string;
	title: string;
	href?: string;
	status: ChapterStatus;
}

export interface Part {
	/** 卷号：序、I、II…… */
	numeral: string;
	title: string;
	en: string;
	chapters: Chapter[];
}

const planned = (no: string, title: string): Chapter => ({ no, title, status: 'planned' });

export const parts: Part[] = [
	{
		numeral: '序',
		title: '基础工具',
		en: 'Foundations',
		chapters: [
			{ no: '0.1', title: '游戏循环与固定时间步长', href: '/foundations/game-loop/', status: 'published' },
			{ no: '0.2', title: '向量与矩阵：点积、叉积在游戏里的用途', href: '/foundations/vectors/', status: 'published' },
			{ no: '0.3', title: '插值与缓动：Lerp、Slerp、Easing、平滑阻尼', href: '/foundations/interpolation/', status: 'published' },
			{ no: '0.4', title: '游戏中的随机数：洗牌、加权随机与伪随机分布', href: '/foundations/random/', status: 'published' },
		],
	},
	{
		numeral: 'I',
		title: '寻路',
		en: 'Pathfinding',
		chapters: [
			{ no: '1.1', title: 'BFS：从洪水填充开始', href: '/pathfinding/bfs/', status: 'published' },
			{ no: '1.2', title: 'Dijkstra：当每一步的代价不同', href: '/pathfinding/dijkstra/', status: 'published' },
			{ no: '1.3', title: 'A*：启发函数与 Open / Closed 表', href: '/pathfinding/astar/', status: 'published' },
			planned('1.4', 'A* 优化：二叉堆、启发权重与路径平滑'),
			planned('1.5', 'JPS 跳点搜索'),
			planned('1.6', '导航网格与漏斗算法'),
			planned('1.7', '流场寻路：RTS 中的大规模单位移动'),
			planned('1.8', '分层寻路 HPA*'),
		],
	},
	{
		numeral: 'II',
		title: '碰撞检测与物理',
		en: 'Collision & Physics',
		chapters: [
			planned('2.1', '基础相交测试：AABB、圆与射线'),
			planned('2.2', '分离轴定理（SAT）'),
			planned('2.3', 'GJK 与 EPA'),
			planned('2.4', '宽相检测：均匀网格与空间哈希'),
			planned('2.5', '宽相检测：四叉树与八叉树'),
			planned('2.6', '宽相检测：BVH 与 Sweep and Prune'),
			planned('2.7', '数值积分：Euler、Verlet 与 RK4'),
			planned('2.8', '碰撞响应与冲量'),
			planned('2.9', '连续碰撞检测：子弹为什么会穿墙'),
		],
	},
	{
		numeral: 'III',
		title: '游戏 AI',
		en: 'Game AI',
		chapters: [
			planned('3.1', '有限状态机与分层状态机'),
			planned('3.2', '行为树'),
			planned('3.3', 'GOAP：目标导向行为规划'),
			planned('3.4', 'Utility AI：效用评分决策'),
			planned('3.5', '转向行为与 Boids 群体模拟'),
			planned('3.6', '影响力图'),
			planned('3.7', 'Minimax 与 Alpha-Beta 剪枝'),
			planned('3.8', '蒙特卡洛树搜索'),
			planned('3.9', '视野与感知：阴影投射'),
		],
	},
	{
		numeral: 'IV',
		title: '程序化生成',
		en: 'Procedural Generation',
		chapters: [
			planned('4.1', '噪声：Value Noise 与 Perlin Noise'),
			planned('4.2', '噪声：Simplex、Worley 与分形布朗运动'),
			planned('4.3', '用噪声生成地形'),
			planned('4.4', '地牢生成：BSP、随机游走、房间与走廊'),
			planned('4.5', '元胞自动机生成洞穴'),
			planned('4.6', '泊松圆盘采样'),
			planned('4.7', 'Voronoi 与 Delaunay 三角剖分'),
			planned('4.8', '波函数坍缩'),
			planned('4.9', 'L-System'),
		],
	},
	{
		numeral: 'V',
		title: '动画与表现',
		en: 'Animation & Rendering',
		chapters: [
			planned('5.1', '贝塞尔曲线与 Catmull-Rom 样条'),
			planned('5.2', '反向动力学：CCD 与 FABRIK'),
			planned('5.3', '相机算法：跟随、死区与震屏'),
			planned('5.4', '粒子系统设计'),
			planned('5.5', '视锥剔除与遮挡剔除'),
		],
	},
	{
		numeral: 'VI',
		title: '架构、性能与网络',
		en: 'Architecture & Networking',
		chapters: [
			planned('6.1', '对象池与内存布局'),
			planned('6.2', 'ECS 与数据导向设计'),
			planned('6.3', '命令模式、录像回放与撤销'),
			planned('6.4', '定点数与确定性计算'),
			planned('6.5', '网络同步：帧同步与状态同步'),
			planned('6.6', '网络同步：客户端预测、回滚与延迟补偿'),
		],
	},
];

export const allChapters = parts.flatMap((part) => part.chapters.map((chapter) => ({ part, chapter })));

export const stats = {
	total: allChapters.length,
	published: allChapters.filter(({ chapter }) => chapter.status === 'published').length,
	parts: parts.length,
};

/** 根据页面路径找到它在目录中的位置，用于文章页眉 */
export function locate(pathname: string) {
	const normalized = pathname.endsWith('/') ? pathname : `${pathname}/`;
	return allChapters.find(({ chapter }) => chapter.href === normalized);
}
