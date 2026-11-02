// 在构建时读取 cpp/ 下的源码，按 `// #region <name>` ... `// #endregion` 截取片段。
// 这样文章里展示的代码永远与经过测试、实际运行在演示里的代码一致。

const sources = import.meta.glob('../../cpp/**/*.{h,hpp,cpp}', {
	query: '?raw',
	import: 'default',
	eager: true,
}) as Record<string, string>;

function lookup(file: string): string {
	const key = `../../cpp/${file}`;
	const source = sources[key];
	if (source === undefined) {
		throw new Error(`[CppSnippet] 找不到源文件 cpp/${file}`);
	}
	return source;
}

function dedent(lines: string[]): string {
	const indents = lines
		.filter((l) => l.trim().length > 0)
		.map((l) => l.match(/^\s*/)![0].length);
	const min = indents.length ? Math.min(...indents) : 0;
	return lines.map((l) => l.slice(min)).join('\n');
}

export function getSnippet(file: string, region?: string): string {
	const source = lookup(file);
	const isMarker = (l: string) => /^\s*\/\/\s*#(end)?region\b/.test(l);

	if (!region) {
		return source.split('\n').filter((l) => !isMarker(l)).join('\n').trimEnd();
	}

	const lines = source.split('\n');
	const start = lines.findIndex((l) => new RegExp(`^\\s*//\\s*#region\\s+${region}\\s*$`).test(l));
	if (start === -1) {
		throw new Error(`[CppSnippet] cpp/${file} 中没有 region "${region}"`);
	}
	const body: string[] = [];
	let depth = 0;
	for (const line of lines.slice(start + 1)) {
		if (/^\s*\/\/\s*#region\b/.test(line)) depth++;
		if (/^\s*\/\/\s*#endregion\b/.test(line)) {
			if (depth === 0) return dedent(body).trimEnd();
			depth--;
		}
		if (!isMarker(line)) body.push(line);
	}
	throw new Error(`[CppSnippet] cpp/${file} 中 region "${region}" 缺少 #endregion`);
}
