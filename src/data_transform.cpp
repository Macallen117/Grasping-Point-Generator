#include "pgfat/data_transform.h"

void eigen2PCL(
  const Eigen::Vector3d &eig,
  const Eigen::Vector3d &norm,
  pcl::PointXYZRGBNormal &pcl_point,
  double r, double g, double b) {
  // transform Eigen::Vector3d point and its normal to pcl::PointXYZRGBNormal
  pcl_point.x = eig(0);
  pcl_point.y = eig(1);
  pcl_point.z = eig(2);
  pcl_point.normal_x = norm(0);
  pcl_point.normal_y = norm(1);
  pcl_point.normal_z = norm(2);
  pcl_point.r = r;
  pcl_point.g = g;
  pcl_point.b = b;
}


void eigen2PCL(
  const Eigen::Vector3d &eig,
  pcl::PointXYZRGBNormal &pcl_point,
  double r, double g, double b) {
  // transform Eigen::Vector3d point to pcl::PointXYZRGBNormal
  pcl_point.x = eig(0);
  pcl_point.y = eig(1);
  pcl_point.z = eig(2);
  pcl_point.r = r;
  pcl_point.g = g;
  pcl_point.b = b;
}


Eigen::Vector3d PCL2eigen(const pcl::PointXYZRGBNormal &pcl) {
  // transform point of pcl::PointXYZRGBNormal to Eigen::Vector3d
  Eigen::Vector3d eig;
  eig(0) = pcl.x;
  eig(1) = pcl.y;
  eig(2) = pcl.z;
  return eig;
}


Eigen::Vector3d PCLNormal2eigen(const pcl::PointXYZRGBNormal &pcl) {
  // transform normal of point of pcl::PointXYZRGBNormal to Eigen::Vector3d
  Eigen::Vector3d eig;
  eig(0) = pcl.normal_x;
  eig(1) = pcl.normal_y;
  eig(2) = pcl.normal_z;
  return eig;
}


void addPoint2Cloud(
  const Eigen::Vector3d &p,
  const Eigen::Vector3d &n_p,
  const std::vector<double> &point_color,
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud) {
  // add Point to point cloud
  pcl::PointXYZRGBNormal pcl_point;
  eigen2PCL(p, n_p, pcl_point,
            point_color[0]*255,
            point_color[1]*255,
            point_color[2]*255);
  cloud->points.push_back(pcl_point);
  cloud->width++;
}


void eigen2Fcl(
    const Eigen::Isometry3d &eigen_input,
    fcl::Transform3d &fcl_output) {
    // transform Eigen::Isometry3d to fcl::Transform3f
    fcl::Matrix3d rotation;
    fcl::Vector3d translation;
    auto &rot = eigen_input.linear();
    auto &trans = eigen_input.translation();
    
    rotation<<rot(0,0), rot(0,1), rot(0,2),
              rot(1,0), rot(1,1), rot(1,2),
              rot(2,0), rot(2,1), rot(2,2);
    translation<<trans(0), trans(1), trans(2);

    //fcl_output.setTransform(rotation,translation);
    fcl_output.linear() = rotation;
    fcl_output.translation() = translation;
    //std::cout<<fcl_output.linear()(0,0)<<std::endl;
    //std::cout<<fcl_output.translation()(1)<<std::endl;
}


