#include <ros/ros.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include "pgfat/visualization.h"
#include "pgfat/grasp_point_generator.h"
#include "pgfat/yaml_config.h"
#include "pgfat/vtk_mesh_utils.h"
#include "pgfat/triangle_plane_data.h"
#include "pgfat/Mesh_preprocessing.h"

using namespace std;
using namespace pcl;


int main(int argc, char** argv)
{

  if (argc<3) return -1;

  YAMLConfig config;
  try
  {
    config.loadConfig(std::string(argv[1]));
  }
  catch(std::exception &e)
  {
      ROS_ERROR("Failed to load yaml file");
  }

  std::string file_name (argv[2]);

  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileSTL(file_name, mesh);  
  std::vector<TrianglePlaneData> triangles = buildTriangleData(mesh);
   
  Mesh_preprocessor mpp;
  GraspPointGenerator gpg;
  Visualizer vis;
  
  mpp.setConfig(config);  
  gpg.setConfig(config);
  vis.setConfig(config);
  
  mpp.setMesh(triangles);
  gpg.setMesh(triangles);
  vis.setMesh(triangles);
   
  mpp.RegionGrowing();   
  
  gpg.setClusters(mpp.clusters);
  gpg.randomPointGenerate();
       
  vis.setProperty(mpp, gpg);
  vis.display_cluster(); 


  return 0;
}
