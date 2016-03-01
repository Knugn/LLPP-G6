// Created for Low Level Parallel Programming 2016
//
// Implements the heatmap functionality. 
//
#include "ped_model.h"

#include <iostream>
#include <string>

#include "ped_model.h"
#include "opencl_utils.h"

bool areHeatmapKernelsSetup = false;
cl::Program clHeatmapProgram;
cl::Kernel clFadeKernel, clIncrementKernel, clScaleKernel, clBlurKernel;
cl::CommandQueue clHeatmapQueue;
cl::Buffer xDesiredBuffer, yDesiredBuffer;
cl::Buffer heatmapBuffer, scaledHeatmapBuffer, blurredHeatmapBuffer, blurFilterBuffer;

const int filterSize = 5;
const int filter[5*5] = {
	1,4,7,4,1,
	4,16,26,16,4,
	7,26,41,26,7,
	4,16,26,16,4,
	1,4,7,4,1
};
const int filterSum = 273;

void Ped::Model::setupHeatmapOpenCl()
{
	/*
	int *hm = (int*) calloc(SIZE*SIZE,sizeof(int));
	int *shm = (int*) malloc(SCALED_SIZE*SCALED_SIZE*sizeof(int));
	int *bhm = (int*) malloc(SCALED_SIZE*SCALED_SIZE*sizeof(int));

	heatmap = (int**) malloc(SIZE*sizeof(int*));

	scaled_heatmap = (int**) malloc(SCALED_SIZE*sizeof(int*));
	blurred_heatmap = (int**) malloc(SCALED_SIZE*sizeof(int*));

	for(int i = 0; i < SIZE; i++)
	{
	heatmap[i] = hm + SIZE*i;
	} 
	for(int i = 0; i < SCALED_SIZE; i++)
	{
	scaled_heatmap[i] = shm + SCALED_SIZE*i;
	blurred_heatmap[i] = bhm + SCALED_SIZE*i;
	} 
	*/
}


void Ped::Model::updateHeatmapOpenClAsync()
{
	const int nAgents = agents.size();

	if (!areHeatmapKernelsSetup) {
		setupOclContext();

		clHeatmapProgram = Ped::OpenClUtils::createProgram(clContext, clDevices, "..\\Libpedsim\\res\\heatmap.cl");

		clFadeKernel = Ped::OpenClUtils::createKernel(clHeatmapProgram, "fade_heatmap");
		clIncrementKernel = Ped::OpenClUtils::createKernel(clHeatmapProgram, "increment_heatmap");
		clScaleKernel = Ped::OpenClUtils::createKernel(clHeatmapProgram, "scale_heatmap");
		clBlurKernel = Ped::OpenClUtils::createKernel(clHeatmapProgram, "blur_heatmap");

		cl_int err = CL_SUCCESS;

		const char* createBufferErrMsg = "Failed to create buffer.";

		xDesiredBuffer = cl::Buffer(clContext, CL_MEM_READ_ONLY, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		yDesiredBuffer = cl::Buffer(clContext, CL_MEM_READ_ONLY, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);

		heatmapBuffer = cl::Buffer(clContext, CL_MEM_READ_WRITE, sizeof(int)*SIZE*SIZE, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		scaledHeatmapBuffer = cl::Buffer(clContext, CL_MEM_READ_WRITE, sizeof(int)*SCALED_SIZE*SCALED_SIZE, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		blurredHeatmapBuffer = cl::Buffer(clContext, CL_MEM_READ_WRITE, sizeof(int)*SCALED_SIZE*SCALED_SIZE, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		blurFilterBuffer = cl::Buffer(clContext, CL_MEM_READ_ONLY, sizeof(int)*CELLSIZE*CELLSIZE, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);

		clHeatmapQueue = cl::CommandQueue(clContext, clDevices[0]);

		const char* writeBuffErrMsg = "Failed to write to device buffers.";
		err = clHeatmapQueue.enqueueFillBuffer(heatmapBuffer, 0, 0, sizeof(int)*SIZE*SIZE);
		Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
		err = clHeatmapQueue.enqueueFillBuffer(scaledHeatmapBuffer, 0, 0, sizeof(int)*SCALED_SIZE*SCALED_SIZE);
		Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
		err = clHeatmapQueue.enqueueFillBuffer(blurredHeatmapBuffer, 0, 0, sizeof(int)*SCALED_SIZE*SCALED_SIZE);
		Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);

		err = clHeatmapQueue.enqueueWriteBuffer(blurFilterBuffer, CL_FALSE, 0, sizeof(int)*5*5, &filter);
		Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);

		areHeatmapKernelsSetup = true;
		std::cout << "Heatmap program and kernels succesfully setup." << std::endl;
	}

	cl_int err = CL_SUCCESS;

	const char* writeBuffErrMsg = "Failed to write to device buffers.";
	err = clHeatmapQueue.enqueueWriteBuffer(xDesiredBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->desX);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
	err = clHeatmapQueue.enqueueWriteBuffer(yDesiredBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->desY);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);

	const char* setArgsErrMsg = "Failed to set kernel arguments.";

	err = clFadeKernel.setArg(0, heatmapBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clFadeKernel.setArg(1, SIZE*SIZE);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);

	err = clIncrementKernel.setArg(0, heatmapBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clIncrementKernel.setArg(1, SIZE);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clIncrementKernel.setArg(2, SIZE);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clIncrementKernel.setArg(3, xDesiredBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clIncrementKernel.setArg(4, yDesiredBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clIncrementKernel.setArg(5, nAgents);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);

	err = clScaleKernel.setArg(0, heatmapBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clScaleKernel.setArg(1, SIZE);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clScaleKernel.setArg(2, SIZE);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clScaleKernel.setArg(3, scaledHeatmapBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clScaleKernel.setArg(4, CELLSIZE);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clScaleKernel.setArg(5, sizeof(int)*256, NULL);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);

	err = clBlurKernel.setArg(0, scaledHeatmapBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clBlurKernel.setArg(1, SCALED_SIZE);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clBlurKernel.setArg(2, SCALED_SIZE);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clBlurKernel.setArg(3, blurFilterBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clBlurKernel.setArg(4, filterSize);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clBlurKernel.setArg(5, filterSize);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clBlurKernel.setArg(6, filterSum);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clBlurKernel.setArg(7, blurredHeatmapBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);


	const char* invokeKernelErrMsg = "Failed to enqueue, invoke or run kernel.";
	
	err = clHeatmapQueue.enqueueNDRangeKernel(clFadeKernel, cl::NullRange, cl::NDRange(SIZE*SIZE), cl::NullRange);
	Ped::OpenClUtils::checkErr(err, invokeKernelErrMsg);
	err = clHeatmapQueue.enqueueNDRangeKernel(clIncrementKernel, cl::NullRange, cl::NDRange(nAgents), cl::NullRange);
	Ped::OpenClUtils::checkErr(err, invokeKernelErrMsg);
	err = clHeatmapQueue.enqueueNDRangeKernel(clScaleKernel, cl::NullRange, cl::NDRange(SCALED_SIZE*SCALED_SIZE), cl::NDRange(CELLSIZE*128));
	Ped::OpenClUtils::checkErr(err, invokeKernelErrMsg);
	err = clHeatmapQueue.enqueueNDRangeKernel(clBlurKernel, cl::NullRange, cl::NDRange(SCALED_SIZE*SCALED_SIZE), cl::NullRange);
	Ped::OpenClUtils::checkErr(err, invokeKernelErrMsg);



	/*
	// Scale the data for visual representation
	for(int y = 0; y < SIZE; y++)
	{
	for(int x = 0; x < SIZE; x++)
	{
	int value = heatmap[y][x];
	for(int cellY = 0; cellY < CELLSIZE; cellY++)
	{
	for(int cellX = 0; cellX < CELLSIZE; cellX++)
	{
	scaled_heatmap[y * CELLSIZE + cellY][x * CELLSIZE + cellX] = value;
	}
	}
	}
	}

	// Weights for blur filter
	const int w[5][5] = {
	{1,4,7,4,1},
	{4,16,26,16,4},
	{7,26,41,26,7},
	{4,16,26,16,4},
	{1,4,7,4,1}
	};

	#define WEIGHTSUM 273
	// Apply gaussian blurfilter		       
	for(int i = 2;i < SCALED_SIZE - 2; i++)
	{
	for(int j = 2;j < SCALED_SIZE - 2; j++)
	{
	int sum = 0;
	for(int k =- 2; k < 3; k++)
	{
	for(int l= -2; l < 3; l++)
	{
	sum += w[2 + k][2 + l] * scaled_heatmap[i + k][j + l];
	}
	}
	int value = sum/WEIGHTSUM;
	blurred_heatmap[i][j] = 0x00FF0000 | value<<24 ;
	}
	}
	*/
}


void Ped::Model::updateHeatmapOpenClWait() {
	clHeatmapQueue.finish();
}
