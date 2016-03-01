#ifndef _opencl_utils_h_
#define _opencl_utils_h_

#include <string>
#include <CL/cl.hpp>

namespace Ped {

	namespace OpenClUtils {

		void checkErr(cl_int err, const char * msg);

		cl::Platform const getDefaultPlatform();
		cl::Device const getDefaultDevice();
		cl::Context const createDefaultDeviceContext();
		
		cl::Program const createProgram(cl::Context context, std::vector<cl::Device> const &devices, std::string const &sourcePath);
		cl::Kernel const createKernel(cl::Program const &program, std::string const &kernelName);

		void printDeviceInfo(cl::Device device);
	}
}

#endif