#ifdef _DEBUG
#define __CL_ENABLE_EXCEPTIONS
#endif

#include "opencl_utils.h"
#include <iostream>

namespace Ped {

	namespace OpenClUtils {
		
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

		void printDeviceInfo(cl::Device device) {
			cl_int err;
			std::cout << "Listing device info ..." << std::endl;

			auto name = device.getInfo<CL_DEVICE_NAME>(&err);
			checkErr(err, "Failed to get device name.");
			std::cout << "Device name: " << name << std::endl;
			
			auto type = device.getInfo<CL_DEVICE_TYPE>(&err);
			checkErr(err, "Failed to get device type.");
			std::string type_human_readable;
			if (type == CL_DEVICE_TYPE_GPU)
				type_human_readable = "GPU";
			else if (type == CL_DEVICE_TYPE_CPU)
				type_human_readable = "CPU";
			else
				type_human_readable = "? (Not CPU or GPU)";
			std::cout << "Device type: " << type_human_readable << std::endl;

			auto device_version = device.getInfo<CL_DEVICE_VERSION>(&err);
			checkErr(err, "Failed to get device version.");
			std::cout << "Device version: " << device_version << std::endl;

			auto opencl_c_version = device.getInfo<CL_DEVICE_OPENCL_C_VERSION>(&err);
			checkErr(err, "Failed to get device OpenCL C version.");
			std::cout << "Device OpenCL C version: " << opencl_c_version << std::endl;

			auto driver_version = device.getInfo<CL_DRIVER_VERSION>(&err);
			checkErr(err, "Failed to get driver version.");
			std::cout << "Driver version: " << driver_version << std::endl;

			auto max_compute_units = device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>(&err);
			checkErr(err, "Failed to get max compute units.");
			std::cout << "Max compute units: " << max_compute_units << std::endl;
		}

		void checkErr(cl_int err, const char * msg) {
			if (err != CL_SUCCESS) {
				std::cerr << "OpenCL ERROR ("<<err<<+"): " << msg  << std::endl;
				std::cout << "Press any key to exit." << std::endl;
				std::getchar();
				exit(EXIT_FAILURE);
			}
		}

	};
}