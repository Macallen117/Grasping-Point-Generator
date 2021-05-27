
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
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h> 

#include "pgfat/geometrics.h"
#include "pgfat/mesh_sampling.h"
#include "pgfat/triangle_plane_data.h"
#include "pgfat/yaml_config.h"


class GraspPointGenerator
{
public:
  void setConfig(const YAMLConfig &config);
  void setMesh(const std::vector <TrianglePlaneData> &triangle_mesh);
  void setClusters(const std::set<std::set<int>> &clusters);
  void randomPointGenerate();
  void setSampleMethode();
  void randomSample(); 
  void eigen2PCL(const Eigen::Vector3d &eig, const Eigen::Vector3d &norm, pcl::PointXYZRGBNormal &pcl, int r, int g, int b);
  void samplePointsInTriangle(TrianglePlaneData & plane);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr candid_sample_cloud_ {new pcl::PointCloud<pcl::PointXYZRGBNormal>};
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr candid_result_cloud {new pcl::PointCloud<pcl::PointXYZRGBNormal>};
 
private:
  std::vector <TrianglePlaneData> planes_;
  std::set<std::set<int>> clusters_;
  YAMLConfig config_;
  

};
