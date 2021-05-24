#pragma once
#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <set>
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


#include "pgfat/triangle_plane_data.h"
#include "pgfat/yaml_config.h"


class Mesh_preprocessor
{
public:
  void setConfig(const YAMLConfig &config);
  void Print_Triangles(const std::vector <TrianglePlaneData> &triangles);
  void RegionGrow(const std::vector <TrianglePlaneData> &triangles);
  bool CheckNormal(const Eigen::Vector3d& Points1,const Eigen::Vector3d& Points2);
  
  std::set<int> find_neibour(const std::vector <TrianglePlaneData> &triangles, const int& seed_index); 
  std::set<std::set<int>> RegionGrowing(const std::vector <TrianglePlaneData> &triangles);
private:
  YAMLConfig config_;


};

