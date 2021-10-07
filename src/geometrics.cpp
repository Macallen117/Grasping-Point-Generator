#include "pgfat/geometrics.h"


double distance2LineSegment(
  const Eigen::Vector3d& A,
  const Eigen::Vector3d& B,
  const Eigen::Vector3d& C) {
  Eigen::Vector3d d = (C - B) / (C - B).norm();
  Eigen::Vector3d v = A - B;
  double t = v.dot(d);
  Eigen::Vector3d P = B + t * d;
  return (P - A).norm();
}


bool calcLinePlaneIntersection(
  const TrianglePlaneData& plane,
  const Eigen::Vector3d& p0,
  const Eigen::Vector3d& u,
  Eigen::Vector3d& p) {
  auto n = plane.normal;
  if (n.dot(u) == 0.0) {  // orth fall
    return false;
  }
  Eigen::Vector3d w = p0 - plane.points[0];
  double s = -n.dot(w) / n.dot(u);

  // inverse direction
  if (s < 0)
    return false;

  p = p0 + s * u;
  return true;
}


bool pointInTriangle(
  const Eigen::Vector3d& p,
  const TrianglePlaneData& plane) {
  auto a = plane.points[0];
  auto b = plane.points[1];
  auto c = plane.points[2];

  // check if point p is not on the plane
  if (abs(plane.normal.dot(p-a))
    + abs(plane.normal.dot(p-b))
    + abs(plane.normal.dot(p-c)) > 1e-3) {  
    return false;
  }
  // Compute vectors
  auto v0 = c - a;
  auto v1 = b - a;
  auto v2 = p - a;

  // Compute dot products
  double dot00 = v0.dot(v0);
  double dot01 = v0.dot(v1);
  double dot02 = v0.dot(v2);
  double dot11 = v1.dot(v1);
  double dot12 = v1.dot(v2);

  // Compute barycentric coordinates
  double inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);
  double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
  double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

  // Check if point is in triangle
  return (u >= 0) && (v >= 0) && (u + v < 1);
}

bool pointInObject(
  const Eigen::Vector3d& p,
  const std::vector <TrianglePlaneData> &triangle_plane,
  std::map<int, std::set<int>> clusters,
  const int &cluster_p) {
  std::map<int, std::set<int>>::iterator cluster_set = clusters.find(cluster_p);
  // std::cout<<"cluster num to be checked: "<<cluster_p<<std::endl;
  for (std::set<int>::iterator vit = cluster_set->second.begin();
    vit != cluster_set->second.end(); vit++) {
    // std::cout<<"triangle in cluster: "<<*vit<<std::endl;
    if (pointInTriangle(p, triangle_plane[*vit]))
      return true;
  }
  return false;
}

Eigen::Vector3d orthogonalVector3d(
  const Eigen::Vector3d& n,
  const Eigen::Vector3d& v0,
  double theta) {
  Eigen::Vector3d v;
  v.setZero();
  v = Eigen::AngleAxisd(theta, n).matrix() * v0;
  return v;
}


Eigen::Vector3d getOrthogonalVector(
  const Eigen::Vector3d& n) {
  Eigen::Vector3d v;

  int max_index = 0;
  int new_index[3];
  double max = 0;
  for (int i = 0; i < 3; i++) {
    if (abs(n(i)) > max) {
      max = abs(n(i));
      max_index = i;
    }
  }

  int loc = 0;
  new_index[2] = max_index;
  for (int i = 0; i < 3; i++) {
    if (i == max_index)
      continue;
    new_index[loc] = i;
    loc++;
  }

  v(new_index[0]) = 1;
  v(new_index[1]) = 1;
  v(new_index[2]) = -(n(new_index[0])+n(new_index[1])) / n(new_index[2]);

  v = v.normalized();

  return v;
}

