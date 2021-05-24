

#pragma once
#include <iostream>
#include <random>

#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pgfat/geometrics.h"
#include "pgfat/mesh_sampling.h"
#include "pgfat/triangle_plane_data.h"
#include "pgfat/yaml_config.h"



class GraspPointGenerator
{
public:

  void setConfig(const YAMLConfig &config);
  void setMesh(const std::vector <TrianglePlaneData> triangle_mesh);
 
private:

  std::vector <TrianglePlaneData> planes_;
  YAMLConfig config_;

};
