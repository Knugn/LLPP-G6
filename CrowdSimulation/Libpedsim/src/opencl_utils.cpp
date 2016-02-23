#ifdef _DEBUG
#define __CL_ENABLE_EXCEPTIONS
#endif

#include "opencl_utils.h"
#include <iostream>

namespace Ped {

	namespace OpenClUtils {
		//
		cl::Platform const getDefaultPlatform() {
			std::vector<cl::Platform> all_platforms;
			cl::Platform::get(&all_platforms);
			if(all_platforms.size()==0){
				throw "No platforms found. Check OpenCL installation!";
			}
			cl::Platform default_platform=all_platforms[0];
			return default_platform;
		}

		cl::Device const getDefaultDevice() {
			cl::Platform default_platform = getDefaultPlatform();
			std::vector<cl::Device> all_devices;
			default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
			if(all_devices.size()==0){
				throw "No devices found. Check OpenCL installation!";
			}
			cl::Device default_device=all_devices[0];
			return default_device;
		}

		cl::Context const createDefaultDeviceContext() {
			cl::Device default_device = getDefaultDevice();
			cl::Context context(default_device);
			return context;
		}

		void checkErr(cl_int err, const char * msg) {
			if (err != CL_SUCCESS) {
				std::cerr << "OpenCL ERROR ("<<err<<+"): " << msg  << std::endl;
				std::cout << "Press any key to exit." << std::endl;
				std::getchar();
				exit(EXIT_FAILURE);
				//throw EXIT_FAILURE;
			}
		}

	};
}