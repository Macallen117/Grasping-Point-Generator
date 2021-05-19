#include "pgfat/grasp_point_generator.h"


void GraspPointGenerator::setConfig(const YAMLConfig &config)
{
  	config_ = config;
}

void GraspPointGenerator::setMesh(const std::vector <TrianglePlaneData> triangle_mesh)
{
  	planes_ = triangle_mesh;
	
}



