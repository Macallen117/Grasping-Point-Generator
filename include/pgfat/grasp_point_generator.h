
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

//#include "btBulletDynamicsCommon.h"

#include "pgfat/geometrics.h"
#include "pgfat/mesh_sampling.h"
#include "pgfat/triangle_plane_data.h"
#include "pgfat/yaml_config.h"


class GraspPointGenerator
{
public:
  void setConfig(const YAMLConfig &config);
  void setMesh(const std::vector <TrianglePlaneData> &triangle_mesh);
  void setClusters(const std::map<int, std::set<int>> &clusters);
  void randomPointGenerate();
  void setSampleMethode();
  void randomSample();  
  void samplePointsInTriangle(TrianglePlaneData & plane);
  void makePair(const Eigen::Vector3d &norm, const Eigen::Vector3d &new_p, const int &cluster_p);
  bool strokeCollisionCheck(const Eigen::Vector3d &p, const Eigen::Vector3d &result_p);
  bool setSecondFinger(const Eigen::Vector3d &p, const Eigen::Vector3d &n_p, const Eigen::Vector3d &result_p, const Eigen::Matrix3d &rotmax, Eigen::Vector3d &p_2, Eigen::Vector3d &result_p_2, const int &cluster_p, const int &cluster_result_p);
  void findCluster(const int &plane_index, int &cluster_result_p);
  void eigen2PCL(const Eigen::Vector3d &eig, const Eigen::Vector3d &norm, pcl::PointXYZRGBNormal &pcl_point, int r, int g, int b);
  void addPoint2Cloud(const pcl::PointXYZRGBNormal &pcl_point,  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr candid_sample_cloud_ {new pcl::PointCloud<pcl::PointXYZRGBNormal>};
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr candid_result_cloud_ {new pcl::PointCloud<pcl::PointXYZRGBNormal>};
 
private:
  std::vector <TrianglePlaneData> planes_;
  std::map<int, std::set<int>> clusters_;
  YAMLConfig config_;
  

};
