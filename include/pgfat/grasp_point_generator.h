
#pragma once
#include <iostream>
#include <random>
#include <algorithm>

#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


#include "pgfat/geometrics.h"
#include "pgfat/mesh_sampling.h"
#include "pgfat/data_structure.h"
#include "pgfat/data_transform.h"
#include "pgfat/yaml_config.h"
#include "pgfat/collision_check_utils.h"


class GraspPointGenerator
{
public:
  void setConfig(const YAMLConfig &config);
  void setMesh(const std::vector <TrianglePlaneData> &triangle_mesh);
  void setClusters(const std::map<int, std::set<int>> &clusters);
  void randomPointGenerate();
  void setSampleMethode();
  void randomSample();  
  void makePair(
    const Eigen::Vector3d &n_p,
    const Eigen::Vector3d &p,
    const std::vector<edge> &bdry);
  bool strokeCollisionCheck(const Eigen::Vector3d &p, const Eigen::Vector3d &result_p);
  bool setSecondFinger(
    const Eigen::Vector3d &p,
    const Eigen::Vector3d &n_p,
    const Eigen::Vector3d &result_p,
    const Eigen::Matrix3d &rotmax,
    Eigen::Vector3d &p_2,
    Eigen::Vector3d &result_p_2,
    const int &cluster_p,
    const int &cluster_result_p);
  void findCluster(
    const int &plane_index,
    int &cluster_result_p);
  void remove_close(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud,
    const double &radius);
  void findbdry(std::vector<int> & Area_index, std::vector<edge> &bdry);
  void setApproachDir(
    const Eigen::Vector3d &p,
    const Eigen::Vector3d &n_p,
    const std::vector<edge> &bdry,
    std::vector<Eigen::Vector3d> &Dir);
  void remove_close2bdry(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud,
    const std::vector<edge> &bdry,
    const double &Distance2bdry);
  bool collisionCheck(GraspData &grasp, const int &mode); 
  bool Areacompare(GraspData &g1, GraspData &g2); 

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr candid_sample_cloud_ 
    {new pcl::PointCloud<pcl::PointXYZRGBNormal>};
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr candid_result_cloud1_ 
    {new pcl::PointCloud<pcl::PointXYZRGBNormal>}; // All collision free grasp points in gripper center
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr candid_result_cloud2_ 
    {new pcl::PointCloud<pcl::PointXYZRGBNormal>}; // All collision free grasp points on finger pads
  
  std::vector <GraspData> grasps_;  // All grasp pose candidates
  std::vector <GraspData> grasp_cand_collision_free1_; // All collision free grasp poses in gripper center
  std::vector <GraspData> grasp_cand_collision_free2_; // All collision free grasp poses on finger pads
  std::vector <GraspData> grasp_cand_in_collision_; // All in collision grasp pose candidates
  CollisionCheck CollisionCheck_;
  
  std::map<int, std::set<int>> clusters_;
  std::map<int, std::vector<edge>> bdrys_;
  std::vector <TrianglePlaneData> planes_;   
  YAMLConfig config_;
 
};
