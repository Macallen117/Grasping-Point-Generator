#pragma once
#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <math.h>
#include <algorithm>


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


#include "pgfat/data_structure.h"
#include "pgfat/yaml_config.h"


class Mesh_preprocessor
{
public:
  void setConfig(const YAMLConfig &config);
  void setMesh(const std::vector <TrianglePlaneData>& triangle_mesh);
  void Print_Triangles();
  void RegionGrow();
  bool CheckNormal(const Eigen::Vector3d& Points1,const Eigen::Vector3d& Points2);
  
  std::set<int> find_neibour(const int& seed_index); 
  void RegionGrowing();
  
  std::map<int, std::set<int>> clusters;
private:
  YAMLConfig config_;
  std::vector <TrianglePlaneData> planes_;
  

};

