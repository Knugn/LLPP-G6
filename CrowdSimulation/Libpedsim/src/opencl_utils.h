#ifndef _opencl_utils_h_
#define _opencl_utils_h_

#include <CL/cl.hpp>

namespace Ped {
	namespace OpenClUtils {
		void checkErr(cl_int err, const char * msg);
		cl::Platform const getDefaultPlatform();
		cl::Device const getDefaultDevice();
		cl::Context const createDefaultDeviceContext();

		void printDeviceInfo(cl::Device device);
	}
}

#endif