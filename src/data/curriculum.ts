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
			{ no: '1.4', title: 'A* 优化：二叉堆、启发权重与路径平滑', href: '/pathfinding/astar-optimization/', status: 'published' },
			{ no: '1.5', title: 'JPS 跳点搜索', href: '/pathfinding/jps/', status: 'published' },
			{ no: '1.6', title: '导航网格与漏斗算法', href: '/pathfinding/navmesh/', status: 'published' },
			{ no: '1.7', title: '流场寻路：RTS 中的大规模单位移动', href: '/pathfinding/flow-field/', status: 'published' },
			{ no: '1.8', title: '分层寻路 HPA*', href: '/pathfinding/hpa/', status: 'published' },
		],
	},
	{
		numeral: 'II',
		title: '碰撞检测与物理',
		en: 'Collision & Physics',
		chapters: [
			{ no: '2.1', title: '基础相交测试：AABB、圆与射线', href: '/collision/intersection/', status: 'published' },
			{ no: '2.2', title: '分离轴定理（SAT）', href: '/collision/sat/', status: 'published' },
			{ no: '2.3', title: 'GJK 与 EPA', href: '/collision/gjk/', status: 'published' },
			{ no: '2.4', title: '宽相检测：均匀网格与空间哈希', href: '/collision/spatial-hash/', status: 'published' },
			{ no: '2.5', title: '宽相检测：四叉树与八叉树', href: '/collision/quadtree/', status: 'published' },
			{ no: '2.6', title: '宽相检测：BVH 与 Sweep and Prune', href: '/collision/bvh/', status: 'published' },
			{ no: '2.7', title: '数值积分：Euler、Verlet 与 RK4', href: '/collision/integration/', status: 'published' },
			{ no: '2.8', title: '碰撞响应与冲量', href: '/collision/impulse/', status: 'published' },
			{ no: '2.9', title: '连续碰撞检测：子弹为什么会穿墙', href: '/collision/ccd/', status: 'published' },
		],
	},
	{
		numeral: 'III',
		title: '游戏 AI',
		en: 'Game AI',
		chapters: [
			{ no: '3.1', title: '有限状态机与分层状态机', href: '/ai/fsm/', status: 'published' },
			{ no: '3.2', title: '行为树', href: '/ai/behavior-tree/', status: 'published' },
			{ no: '3.3', title: 'GOAP：目标导向行为规划', href: '/ai/goap/', status: 'published' },
			{ no: '3.4', title: 'Utility AI：效用评分决策', href: '/ai/utility/', status: 'published' },
			{ no: '3.5', title: '转向行为与 Boids 群体模拟', href: '/ai/steering/', status: 'published' },
			{ no: '3.6', title: '影响力图', href: '/ai/influence-map/', status: 'published' },
			{ no: '3.7', title: 'Minimax 与 Alpha-Beta 剪枝', href: '/ai/minimax/', status: 'published' },
			{ no: '3.8', title: '蒙特卡洛树搜索', href: '/ai/mcts/', status: 'published' },
			{ no: '3.9', title: '视野与感知：阴影投射', href: '/ai/fov/', status: 'published' },
		],
	},
	{
		numeral: 'IV',
		title: '程序化生成',
		en: 'Procedural Generation',
		chapters: [
			{ no: '4.1', title: '噪声：Value Noise 与 Perlin Noise', href: '/procgen/noise/', status: 'published' },
			{ no: '4.2', title: '噪声：Simplex、Worley 与分形布朗运动', href: '/procgen/fractal/', status: 'published' },
			{ no: '4.3', title: '用噪声生成地形', href: '/procgen/terrain/', status: 'published' },
			{ no: '4.4', title: '地牢生成：BSP、随机游走、房间与走廊', href: '/procgen/dungeon/', status: 'published' },
			{ no: '4.5', title: '元胞自动机生成洞穴', href: '/procgen/cave/', status: 'published' },
			{ no: '4.6', title: '泊松圆盘采样', href: '/procgen/poisson/', status: 'published' },
			{ no: '4.7', title: 'Voronoi 与 Delaunay 三角剖分', href: '/procgen/voronoi/', status: 'published' },
			{ no: '4.8', title: '波函数坍缩', href: '/procgen/wfc/', status: 'published' },
			{ no: '4.9', title: 'L-System', href: '/procgen/lsystem/', status: 'published' },
		],
	},
	{
		numeral: 'V',
		title: '动画与表现',
		en: 'Animation & Rendering',
		chapters: [
			{ no: '5.1', title: '贝塞尔曲线与 Catmull-Rom 样条', href: '/animation/spline/', status: 'published' },
			{ no: '5.2', title: '反向动力学：CCD 与 FABRIK', href: '/animation/ik/', status: 'published' },
			{ no: '5.3', title: '相机算法：跟随、死区与震屏', href: '/animation/camera/', status: 'published' },
			{ no: '5.4', title: '粒子系统设计', href: '/animation/particles/', status: 'published' },
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
