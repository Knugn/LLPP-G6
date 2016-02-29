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
cl::Kernel clHeatmapKernel;
cl::CommandQueue clHeatmapQueue;
cl::Buffer heatmapBuffer, scaledHeatmapBuffer, blurredHeatmapBuffer;


void setupHeatmapProgram(cl::Device clDevice, cl::Context clContext) {
	/*
	std::string kernelPath = "..\\Libpedsim\\res\\heatmap.cl";
	cl::Program::Sources sources = Ped::OpenClUtils::loadSource(kernelPath);
	clHeatmapProgram = cl::Program(clContext, sources);
	std::vector<cl::Device> devices;
	devices.push_back(clDevice);
	cl_int err = clHeatmapProgram.build(devices);
	Ped::OpenClUtils::checkErr(err, ("Failed to build program. Build log: \n" + 
		clHeatmapProgram.getBuildInfo<CL_PROGRAM_BUILD_LOG>(clDevice)).c_str());
		*/
	std::cout << "Successfully built heatmap program." << std::endl;
}

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
	/*
  for(int x = 0; x < SIZE; x++)
  {
    for(int y = 0; y < SIZE; y++)
    {
      // heat fades
      heatmap[y][x] *= 0.80;
    }
  }
  
  // Count how many agents want to go to each location
  for(int i = 0; i < agents.size(); i++)
  {
    Ped::Tagent* agent = agents[i];
    int x = agent->getDesiredX();
    int y = agent->getDesiredY();

    if(x < 0 || x >= SIZE || y < 0 || y >= SIZE)
    {
      continue;
    }
    
    // intensify heat for better color results
    heatmap[y][x] += 40;

  }

  for(int x = 0; x < SIZE; x++)
  {
    for(int y = 0; y < SIZE; y++)
    {
      heatmap[y][x]=  heatmap[y][x] < 255 ?heatmap[y][x]:255;
    }
  }
  
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
