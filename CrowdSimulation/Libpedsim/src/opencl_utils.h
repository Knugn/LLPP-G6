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
		
		cl::Program const createProgram(cl::Context context, std::vector<cl::Device> &devices, std::string const &sourcePath);

		void printDeviceInfo(cl::Device device);
	}
}

#endif