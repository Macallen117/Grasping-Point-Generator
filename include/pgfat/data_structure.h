#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <set>



struct TrianglePlaneData {
  Eigen::Vector3d normal;
  std::vector < Eigen::Vector3d > points {3};
  std::set <int> neighbors;
  double area;

  friend std::ostream & operator << (std::ostream &out, const TrianglePlaneData &d) {
    out << "normal: " << d.normal.transpose() << std::endl
        << "area: " << d.area << std::endl
        << "points: " << std::endl;
    for (const auto &point : d.points) {
      out << "        " << point.transpose() << std::endl;
    }
    return out;
  }
};



struct GraspData {
  std::vector<Eigen::Vector3d> points;
  Eigen::Isometry3d hand_transform;  // 4x4 Rotation matrix
  double dist;  // distance between p and result_p

  friend std::ostream & operator << (std::ostream &out, const GraspData &d) {
    out << "transform: " << std::endl
        << d.hand_transform.matrix() << std::endl
        << "points: " << std::endl;
    for (const auto &point : d.points) {
      out << "        " << point.transpose() << std::endl;
    }
    return out;
  }

  double getDist() {
    dist = (points[0] - points[1]).norm();
    return dist;
  }
};
