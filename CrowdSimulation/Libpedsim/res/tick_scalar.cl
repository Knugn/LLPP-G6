
__kernel void agent_move(__global int * xPos, 
						 __global int * yPos,
						 __constant int * xDest,
						 __constant int * yDest,
						 __constant float * rDest,
						 const int nAgents)
{
	int gid = get_global_id(0);
	if (gid >= nAgents)
		return;
	int dx = xDest[gid] - xPos[gid];
	int dy = yDest[gid] - yPos[gid];
	float dist = sqrt(convert_float(dx*dx+dy*dy));
	if (dist < rDest[gid])
		return;
	dx = round(dx/dist);
	dy = round(dy/dist);
	xPos[gid] += dx;
	yPos[gid] += dy;
}
