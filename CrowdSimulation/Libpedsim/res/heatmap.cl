
__kernel void update_heatmap(__global int * heatmap,
							 __constant int * xDesired,
							 __constant int * yDesired,
							 const int wHeatmap,
							 const int nAgents)
{
	int gid = get_global_id(0);
	if (gid >= nAgents)
		return;

}

__kernel void scale_heatmap(__const int * srcHeatmap,
							__global int * dstHeatmap)
{

}

__kernel void blur_heatmap(__const int * srcHeatmap,
						   __global int * dstHeatmap)
{

}
