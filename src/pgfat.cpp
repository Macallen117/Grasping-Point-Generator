#include <ros/ros.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <ros/package.h>

#include "pgfat/visualization.h"
#include "pgfat/grasp_point_generator.h"
#include "pgfat/yaml_config.h"
#include "pgfat/vtk_mesh_utils.h"
#include "pgfat/data_structure.h"
#include "pgfat/Mesh_preprocessing.h"


int main(int argc, char** argv) {
  if (argc < 2)
    return -1;
  
  std::string PKG_path = ros::package::getPath("pgfat");
  
  YAMLConfig config;
  try {
    config.loadConfig(PKG_path + "/config/options.yaml");
  }
  catch(std::exception &e) {
      ROS_ERROR("Failed to load yaml file");
  }
     
  
  std::string object_name(argv[1]);
  object_name += ".stl";
  
  // load the object mesh to be analysed
  std::string mesh_file = PKG_path + "/meshes/Motor_part/" + object_name;
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileSTL(mesh_file, mesh);
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
  //vis.display_cluster();
  vis.display_grasp(mesh);

  return 0;
}
