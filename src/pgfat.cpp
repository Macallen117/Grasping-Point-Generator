#include <ros/ros.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>


#include "pgfat/grasp_point_generator.h"
#include "pgfat/yaml_config.h"
#include "pgfat/vtk_mesh_utils.h"
#include "pgfat/triangle_plane_data.h"
#include "pgfat/Mesh_preprocessing.h"
#include "pgfat/visualization.h"

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
  Visualizer vis;
  GraspPointGenerator gpg;
  
  mpp.setConfig(config);
  vis.setConfig(config);
  gpg.setConfig(config);
  
  //mpp.Print_Triangles(triangles);
  //mpp.RegionGrow(triangles);
  std::set<std::set<int>> clusters;
  clusters = mpp.RegionGrowing(triangles);
  
  
  vis.display_initial(mesh);
     
  
  //gpg.display_initial(mesh);
  //gpg.setMesh(triangles);
  //gpg.generate();
  //gpg.display(mesh);


  return 0;
}
