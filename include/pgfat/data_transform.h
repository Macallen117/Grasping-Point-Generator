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

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/BV/BV.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/shape/geometric_shapes.h>


#include "pgfat/geometrics.h"
#include "pgfat/mesh_sampling.h"
#include "pgfat/data_structure.h"
#include "pgfat/yaml_config.h"




void eigen2PCL(
  const Eigen::Vector3d &eig,
  const Eigen::Vector3d &norm,
  pcl::PointXYZRGBNormal &pcl_point,
  double r, double g, double b);


void eigen2PCL(
  const Eigen::Vector3d &eig,
  pcl::PointXYZRGBNormal &pcl_point,
  double r, double g, double b);
  
void eigen2Fcl(
    const Eigen::Isometry3d &eigen_input,
    fcl::Transform3f &fcl_output);


Eigen::Vector3d PCL2eigen(const pcl::PointXYZRGBNormal &pcl);


Eigen::Vector3d PCLNormal2eigen(const pcl::PointXYZRGBNormal &pcl);


void addPoint2Cloud(
  const Eigen::Vector3d &p,
  const Eigen::Vector3d &n_p,
  const std::vector<double> &point_color,
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud);
