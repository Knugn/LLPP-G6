
#include <iostream>
#include <string>

#include "ped_model.h"
#include "opencl_utils.h"

bool isMoveKernelSetup = false;
cl::Program clMoveProgram;
cl::Kernel clMoveKernel;
cl::CommandQueue clMoveQueue;
cl::Buffer xPosBuffer, yPosBuffer, xDestBuffer, yDestBuffer, rDestBuffer;


void Ped::Model::tick_opencl() {
	int nAgents = agents.size();
	for (int i = 0; i < nAgents; i++) {
		agents[i]->getNextDestinationNormal();
	}

	if (!isMoveKernelSetup) {
		setupOclContext();

		clMoveProgram = Ped::OpenClUtils::createProgram(clContext, clDevices, "..\\Libpedsim\\res\\tick_scalar.cl");

		cl_int err;

		std::string kernelName = "agent_move";
		clMoveKernel = cl::Kernel(clMoveProgram, kernelName.c_str(), &err);
		Ped::OpenClUtils::checkErr(err, ("Failed to locate kernel entry function \"" + kernelName + "\"").c_str());

		const char* createBufferErrMsg = "Failed to create buffer.";
		xPosBuffer = cl::Buffer(clContext, CL_MEM_READ_WRITE, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		yPosBuffer = cl::Buffer(clContext, CL_MEM_READ_WRITE, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		xDestBuffer = cl::Buffer(clContext, CL_MEM_READ_ONLY, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		yDestBuffer = cl::Buffer(clContext, CL_MEM_READ_ONLY, sizeof(int)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);
		rDestBuffer = cl::Buffer(clContext, CL_MEM_READ_ONLY, sizeof(float)*nAgents, &err);
		Ped::OpenClUtils::checkErr(err, createBufferErrMsg);

		clMoveQueue = cl::CommandQueue(clContext, clDevices[0]);

		isMoveKernelSetup = true;
	}

	cl_int err;

	const char* writeBuffErrMsg = "Failed to write to device buffers.";
	err = clMoveQueue.enqueueWriteBuffer(xPosBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->x);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
	err = clMoveQueue.enqueueWriteBuffer(yPosBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->y);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
	err = clMoveQueue.enqueueWriteBuffer(xDestBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->destX);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
	err = clMoveQueue.enqueueWriteBuffer(yDestBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->destY);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);
	err = clMoveQueue.enqueueWriteBuffer(rDestBuffer, CL_FALSE, 0, sizeof(float)*nAgents, agents[0]->destR);
	Ped::OpenClUtils::checkErr(err, writeBuffErrMsg);

	const char* setArgsErrMsg = "Failed to set kernel arguments.";
	err = clMoveKernel.setArg(0, xPosBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clMoveKernel.setArg(1, yPosBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clMoveKernel.setArg(2, xDestBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clMoveKernel.setArg(3, yDestBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clMoveKernel.setArg(4, rDestBuffer);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);
	err = clMoveKernel.setArg(5, nAgents);
	Ped::OpenClUtils::checkErr(err, setArgsErrMsg);

	err = clMoveQueue.enqueueNDRangeKernel(clMoveKernel, cl::NullRange, cl::NDRange(nAgents), cl::NullRange);
	Ped::OpenClUtils::checkErr(err, "Failed to enqueue, invoke or run kernel.");

	const char* readBuffErrMsg = "Failed to read buffers back to host.";
	err = clMoveQueue.enqueueReadBuffer(xPosBuffer, CL_FALSE, 0, sizeof(int)*nAgents, agents[0]->x);
	Ped::OpenClUtils::checkErr(err, readBuffErrMsg);
	err = clMoveQueue.enqueueReadBuffer(yPosBuffer, /*blocking=*/CL_TRUE, 0, sizeof(int)*nAgents, agents[0]->y); 
	Ped::OpenClUtils::checkErr(err, readBuffErrMsg);
}
