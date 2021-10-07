
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <pcl/common/intersections.h>
#include <vector>
#include <map>

#include "pgfat/data_structure.h"

double distance2LineSegment(
  const Eigen::Vector3d& A,
  const Eigen::Vector3d& B,
  const Eigen::Vector3d& C);
  
bool calcLinePlaneIntersection(
  const TrianglePlaneData& plane, 
  const Eigen::Vector3d& p0, 
  const Eigen::Vector3d& u,
  Eigen::Vector3d& p);

bool pointInTriangle(
  const Eigen::Vector3d& p,
  const TrianglePlaneData& plane);
bool pointInObject(
  const Eigen::Vector3d& p,
  const std::vector <TrianglePlaneData> &triangle_plane,
  std::map<int, std::set<int>> clusters,
  const int &cluster_p);

Eigen::Vector3d orthogonalVector3d(
  const Eigen::Vector3d&  n,
  const Eigen::Vector3d&  v0,
  double theta);
Eigen::Vector3d getOrthogonalVector(const Eigen::Vector3d&  n);
