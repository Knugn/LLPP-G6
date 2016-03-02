
#include "ped_model.h"

#include <iostream>
#include <string>
#include <cmath>

#include "ped_model.h"
#include "opencl_utils.h"

bool areHeatmapKernelsSetup = false;
cl::Program clHeatmapProgram;
cl::Kernel clFadeKernel, clIncrementKernel, clScaleKernel, clBlurKernel;
cl::CommandQueue clHeatmapQueue;
cl::Buffer xDesiredBuffer, yDesiredBuffer;
cl::Buffer heatmapBuffer, scaledHeatmapBuffer, blurredHeatmapBuffer, blurFilterBuffer;
int blurTileSize, nBlurTilesX, nBlurTilesY;

const int filterSize = 5;
const int filter[5*5] = {
	1,4,7,4,1,
	4,16,26,16,4,
	7,26,41,26,7,
	4,16,26,16,4,
	1,4,7,4,1
};
const int filterSum = 273;


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

		err = clHeatmapQueue.enqueueWriteBuffer(blurFilterBuffer, CL_FALSE, 0, sizeof(int)*filterSize*filterSize, &filter);
		Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);

		blurTileSize = min(
			(int)sqrt(clDevices[0].getInfo<CL_DEVICE_LOCAL_MEM_SIZE>() / sizeof(int)),
			(int)sqrt(clDevices[0].getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>())
			);
		nBlurTilesX = (int) ceil(SCALED_SIZE / (blurTileSize-(filterSize-1)));
		nBlurTilesY = nBlurTilesX;
		std::cout << "Blur tile size: " << blurTileSize << std::endl;
		std::cout << "Number of blur tiles along x: " << nBlurTilesX << std::endl;
		std::cout << "Number of blur tiles along y: " << nBlurTilesY << std::endl;

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
	err = clBlurKernel.setArg(8, sizeof(int)*blurTileSize*blurTileSize, NULL);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);


	const char* invokeKernelErrMsg = "Failed to enqueue, invoke or run kernel.";

	err = clHeatmapQueue.enqueueNDRangeKernel(clFadeKernel, cl::NullRange, cl::NDRange(SIZE*SIZE), cl::NullRange);
	Ped::OpenClUtils::checkErr(err, invokeKernelErrMsg);
	err = clHeatmapQueue.enqueueNDRangeKernel(clIncrementKernel, cl::NullRange, cl::NDRange(nAgents), cl::NullRange);
	Ped::OpenClUtils::checkErr(err, invokeKernelErrMsg);
	err = clHeatmapQueue.enqueueNDRangeKernel(clScaleKernel, cl::NullRange, cl::NDRange(SCALED_SIZE*SCALED_SIZE), cl::NDRange(CELLSIZE*128));
	Ped::OpenClUtils::checkErr(err, invokeKernelErrMsg);
	err = clHeatmapQueue.enqueueNDRangeKernel(clBlurKernel, cl::NullRange, 
		cl::NDRange(blurTileSize*nBlurTilesX,blurTileSize*nBlurTilesY),
		cl::NDRange(blurTileSize,blurTileSize));
	Ped::OpenClUtils::checkErr(err, invokeKernelErrMsg);


	const char* readBuffErrMsg = "Failed to read buffers back to host.";
	err = clHeatmapQueue.enqueueReadBuffer(blurredHeatmapBuffer, CL_FALSE, 0, sizeof(int)*SCALED_SIZE*SCALED_SIZE, *blurred_heatmap);
	Ped::OpenClUtils::checkErr(err, readBuffErrMsg);

}


void Ped::Model::updateHeatmapOpenClWait() {
	clHeatmapQueue.finish();
}
