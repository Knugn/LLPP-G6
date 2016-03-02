
__kernel void fade_heatmap(__global int * heatmap, const int N)
{
	int gid = get_global_id(0);
	if (gid >= N)
		return;
	heatmap[gid] = (heatmap[gid] * 4) / 5;
}

__kernel void increment_heatmap(__global int * heatmap, 
							 const int wHeatmap, 
							 const int hHeatmap,
							 __constant int * xDesired,
							 __constant int * yDesired,
							 const int nAgents)
{
	int gid = get_global_id(0);
	if (gid >= nAgents)
		return;
	int x = xDesired[gid];
	int y = yDesired[gid];
	if (x < 0 || wHeatmap <= x || y < 0 || hHeatmap <= y)
		return;

	__global int * p = heatmap + y * wHeatmap + x;
	int old = *p;
	if (old >= 255)
		return;

	// Atomically increment heatmap at p.
	old = atomic_add(p, 40);
	if (old + 40 > 255)
		atomic_and(p,0xFF);
}

__kernel void scale_heatmap(__global int * srcHeatmap,
							const int wSrcHeatmap, 
							const int hSrcHeatmap,
							__global int * dstHeatmap,
							const int cellSize,
							__local int * srcLine)
{
	const int gid = get_global_id(0);
	if (gid >= wSrcHeatmap*hSrcHeatmap*cellSize*cellSize)
		return;
	const int lid = get_local_id(0);
	const int localSize = get_local_size(0);

	if (lid * cellSize < localSize) {
		int temp = (gid-lid + lid*cellSize);
		int tempx = temp % (wSrcHeatmap*cellSize);
		int tempy = temp / (wSrcHeatmap*cellSize);
		tempx /= cellSize;
		tempy /= cellSize;
		srcLine[lid] = srcHeatmap[tempy*wSrcHeatmap+tempx];
	}
	barrier(CLK_LOCAL_MEM_FENCE);

	int localIdx = lid / cellSize;
	dstHeatmap[gid] = srcLine[localIdx];
}

__kernel void blur_heatmap(__global int * srcHeatmap,
						   const int wSrcHeatmap, 
						   const int hSrcHeatmap,
						   __constant int * filter,
						   const int wFilter,
						   const int hFilter,
						   const int filterSum,
						   __global int * dstHeatmap,
						   __local int * localTile)
{
	/*
	const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
	if (gidx < 2 || gidx >= wSrcHeatmap - 2 ||
		gidy < 2 || gidy >= hSrcHeatmap - 2)
		return;

	int value = srcHeatmap[gidy*wSrcHeatmap+gidx];
	dstHeatmap[gidy*wSrcHeatmap+gidx] = 0x00FF0000 | value << 24;
	
	*/
	const int lidx = get_local_id(0);
	const int lidy = get_local_id(1);
	const int lwsx = get_local_size(0);
	const int lwsy = get_local_size(1);
	const int wgidx = get_group_id(0);
	const int wgidy = get_group_id(1);
	const int xOffset = 0;
	const int yOffset = 0;
	const int px = xOffset + wgidx*(lwsx-(wFilter-1)) + lidx;
	const int py = yOffset + wgidy*(lwsy-(hFilter-1)) + lidy;
	if (px < 0 || px >= wSrcHeatmap ||
		py < 0 || py >= hSrcHeatmap )
		return;

	localTile[lidy*lwsx+lidx] = srcHeatmap[py*wSrcHeatmap+px];

	const int padx = wFilter / 2;
	const int pady = hFilter / 2;
	if (lidx < padx || lidx >= lwsx - padx ||
		lidy < pady || lidy >= lwsy - pady ||
		px < padx || px >= wSrcHeatmap - padx ||
		py < pady || py >= hSrcHeatmap - pady )
		return;

	barrier(CLK_LOCAL_MEM_FENCE);

	int sum = 0;
	for (int fy = 0; fy < hFilter; fy++) {
		for (int fx = 0; fx < wFilter; fx++) {
			sum += localTile[(lidy-pady+fy)*lwsx+(lidx-padx+fx)] * filter[fy*wFilter+fx];
		}
	}
	const int value = sum / filterSum;
	dstHeatmap[py*wSrcHeatmap+px] = 0x00FF0000 | value << 24;
}
