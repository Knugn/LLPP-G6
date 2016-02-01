CXX_ = g++
LDFLAGS_ = -lpthread -fopenmp -shared -Wl,-soname,libpedsim.so

NVCC = nvcc
SOURCES_CXX = ped_vector.cpp ped_waypoint.cpp ped_agent.cpp ped_model.cpp heatmap_seq.cpp
OBJECTS_CXX = $(SOURCES_CXX:.cpp=.cxx.o)

SOURCES_CU = tick_cuda.cu
OBJECTS_CU = $(SOURCES_CU:.cu=.cu.o)

CXXFLAGS_ = -lpthread -fopenmp -g -O3 -fno-inline -fPIC

# Compile everything with nvcc. nvcc calls system default c++ compiler for the host code.
# Make shared library directly with default c+++ compiler, makes format compatability more likely, and life more easy.
CXX = $(NVCC)
CXXFLAGS = -arch sm_30 -rdc=true --compiler-options "$(CXXFLAGS_)"
LDFLAGS = --shared #--compiler-options "$(LDFLAGS_)"
OBJECTS =  prelinked_objects.o  $(OBJECTS_CU) $(OBJECTS_CXX)


libpedsim.so: $(OBJECTS)
	$(CXX_) $(LDFLAGS_) $(OBJECTS) -o libpedsim.so 

%.cu.o: %.cu
	$(NVCC) $(CXXFLAGS) -c $< -o $@ 

%.cxx.o: %.cpp
	$(CXX) $(CXXFLAGS)  -c $< -o $@ 

# We need to do some linking beforehand, so that non-cuda compilers can link to the lib
# If the program were using nvcc, this step would be unneccesary
prelinked_objects.o:  $(OBJECTS_CU) $(OBJECTS_CXX)
	$(NVCC) $(CXXFLAGS) -dlink  $(OBJECTS_CU) $(OBJECTS_CXX) -o $@

# This removes a circular dependency from implicit make rules.
%.cu:

clean:
	rm -f *.o *.so *~ \#*
