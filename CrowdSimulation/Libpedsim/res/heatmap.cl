
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
	// At the most 9 agents will fight for a desired position.
	// Also, 7*40 = 280 > 255, so no more than 7 attempts should have to be made at incrementing.
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

	//__local int * srcLine;
	if (lid * cellSize < localSize) {
		int temp = (gid-lid + lid*cellSize);
		int tempx = temp % (wSrcHeatmap*cellSize);
		//int temp -= tempx;
		int tempy = temp / (wSrcHeatmap*cellSize);
		tempx /= cellSize;
		tempy /= cellSize;
		srcLine[lid] = srcHeatmap[tempy*wSrcHeatmap+tempx];
	}
	// barrier here plz
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
						   __global int * dstHeatmap)
{
	const int gidx = get_global_id(0);
	const int gidy = get_global_id(1);
	if (gidx < 2 || gidx >= wSrcHeatmap - 2 ||
		gidy < 2 || gidy >= hSrcHeatmap - 2)
		return;
	dstHeatmap[gidy*wSrcHeatmap+gidx] = srcHeatmap[gidy*wSrcHeatmap+gidx];
	//const int lidx = get_local_id(0);
	//const int lidy = get_local_id(1);

}
