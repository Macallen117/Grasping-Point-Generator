#include "pgfat/grasp_point_generator.h"


void GraspPointGenerator::setConfig(const YAMLConfig &config) {
  config_ = config;
  CollisionCheck_.gripper_model_.setParams(
    config_.gripper_params[0],
    config_.gripper_params[1],
    config_.gripper_params[2],
    config_.gripper_params[3],
    config_.gripper_params[4],
    config_.gripper_params[5],
    config_.gripper_params[6]);
}

void GraspPointGenerator::setMesh(
  const std::vector <TrianglePlaneData> &triangle_mesh) {
  planes_ = triangle_mesh;
  CollisionCheck_.loadMesh(triangle_mesh);
}

void GraspPointGenerator::setClusters(
  const std::map<int,
  std::set<int>> &clusters) {
  clusters_ = clusters;
}
void GraspPointGenerator::randomPointGenerate() {
  setSampleMethode();
}

void GraspPointGenerator::setSampleMethode() {
  if (config_.point_generation_method == "random_sample") {
    randomSample();
  }
}

void GraspPointGenerator::randomSample() {
  // sample point randomly in every cluster we segmented
  // The number of points to be sampled is currently
  // equal to the total area of all the triangles in the cluster

  // totalArea: total area of all the triangles in one cluster
  // cumulativeAreas: sum of area as we continuing adding area of triangle
  // Area_index: the current triangle ID to be added
  // random_point_num: number of randomly sampled points

  // alfter sampling, make point pair for every point we sampled using makePair(n, p)

  std::default_random_engine generator;
  std::uniform_real_distribution<double> mesh_distribution(0.0, 1.0);

  for (std::map<int, std::set<int>>::iterator it = clusters_.begin();
    it != clusters_.end(); it++) {
    std::cout << "cluster now: " << it->first << "/"
              << clusters_.size() << std::endl;
    double totalArea = 0;
    int random_point_num;
    std::vector<double> cumulativeAreas;
    std::vector<int> Area_index;

    for (std::set<int>::iterator vit = it->second.begin();
      vit != it->second.end(); vit++) {
      totalArea += planes_[*vit].area;
      cumulativeAreas.push_back(totalArea);
      Area_index.push_back(*vit);
    }
    random_point_num = totalArea;
    for (std::size_t i = 0; i < random_point_num; i++) {
      Eigen::Vector3d p;
      Eigen::Vector3d n(0, 0, 0);
      // r :(0-1)*totalArea   r1, r2: 0-1
      double r = mesh_distribution(generator) * totalArea;
      double r1 = mesh_distribution(generator);
      double r2 = mesh_distribution(generator);
      randPSurface(planes_, Area_index, cumulativeAreas, totalArea, p, n, r, r1, r2);
      makePair(n, p);
    }
  }
  std::cout << "candid_sample_cloud_ "
            << candid_sample_cloud_->points.size()/2
            << std::endl;
  std::cout << "candid_result_cloud_ "
            << candid_result_cloud_->points.size()/2
            << std::endl;
  std::cout << "grasp_cand_collision_free_ "
            << grasp_cand_collision_free_.size()
            << std::endl;
}

void GraspPointGenerator::makePair(
  const Eigen::Vector3d &n_p,
  const Eigen::Vector3d &p) {
  // firstly find the correspoding point for a given point to make a point pair
  // and then check if collide

  for (auto & plane : planes_) {
    auto & n_result_p = plane.normal;
    // opposite dir tolerance
    if ((n_p+n_result_p).norm() < config_.Theta_parl) {
      Eigen::Vector3d result_p;
      calcLinePlaneIntersection(plane, p, -n_p, result_p);

      if (!pointInTriangle(result_p, plane))
        continue;
      if (!strokeCollisionCheck(p, result_p))
        continue;

      //Eigen::Vector3d axis = (p - result_p).normalized();
      Eigen::Vector3d axis = n_p;
      double theta;
      Eigen::Matrix3d rotmax;
      for (int i = 0; i < config_.N_da; i++) {
        theta = 360.0 / config_.N_da * i;
        rotmax = Eigen::AngleAxisd(theta, axis).matrix();

        // add this point pair to sampled point cloud
        // use a number times n_p for better visual
        addPoint2Cloud(p, n_p, candid_sample_cloud_);
        addPoint2Cloud(result_p, n_result_p, candid_sample_cloud_);

        // create graspdata
        Eigen::Vector3d orth = getOrthogonalVector(axis);
        Eigen::Vector3d approach_direction = rotmax * orth;
        GraspData gd;
        gd.hand_transform.linear().col(0) = approach_direction;  // X
        gd.hand_transform.linear().col(1) = n_p.cross(approach_direction);  // Y = Z cross X
        gd.hand_transform.linear().col(2) = n_p;  // Z
        gd.hand_transform.translation() = (p + result_p) / 2;  // tip position
        gd.points.push_back(p);
        gd.points.push_back(result_p);
        grasps_.push_back(gd);

        // check collision for every graspdata
        if (collisionCheck(gd)) {
          // after collision check add this point pair to result point cloud
          addPoint2Cloud(gd.points[0], gd.hand_transform.linear().col(2), candid_result_cloud_);
          addPoint2Cloud(gd.points[1], gd.hand_transform.linear().col(2), candid_result_cloud_);
          // add collision free graspdata to data set
          grasp_cand_collision_free_.push_back(gd);
        } else {
          grasp_cand_in_collision_.push_back(gd);
        }
      }
    }
  }
}


bool GraspPointGenerator::collisionCheck(GraspData &grasp) {
  if (CollisionCheck_.isCollide(grasp.hand_transform,
                                grasp.getDist()/2 + 0.001))
    return true;
  else
    return false;
}



bool GraspPointGenerator::strokeCollisionCheck(
  const Eigen::Vector3d &p,
  const Eigen::Vector3d &result_p) {
  // the distance of one point pair must be smaller than 2 * stroke per jaw

  auto dist = result_p - p;
  if (abs(dist.norm()) < config_.Stroke_per_jaw * 2)
    return true;
  else
    return false;
}


bool GraspPointGenerator::setSecondFinger(
  const Eigen::Vector3d &p,
  const Eigen::Vector3d &n_p,
  const Eigen::Vector3d &result_p,
  const Eigen::Matrix3d &rotmax,
  Eigen::Vector3d &p_2,
  Eigen::Vector3d &result_p_2,
  const int &cluster_p,
  const int &cluster_result_p) {
  // p: first point in the point pair
  // n_p: normal of p
  // result_p: the second point in the point pair
  // cluster_p: cluster index of p
  // cluster_result_p: cluster index of result_p

  // orth: find the orthogonal vector of point normal
  // dir: rotmax * orth for calculating the second point
  // p_2: first point of the second point pair
  // result_p_2: second point of the second point pair

  // and then using pointInObject() to check if p_2 and result_p_2
  // lie on the surface of object

  Eigen::Vector3d orth = getOrthogonalVector(n_p);
  Eigen::Vector3d dir = rotmax * orth;
  p_2 = p + config_.gripper_fingers_dist * dir;
  result_p_2 = result_p + config_.gripper_fingers_dist * dir;

  if ((pointInObject(p_2, planes_, clusters_, cluster_p))
    &&(pointInObject(result_p_2, planes_, clusters_, cluster_result_p))) {
    return true;
  }
  return false;
}


void GraspPointGenerator::findCluster(
  const int &plane_index,
  int &cluster_result_p) {
  // find cluster index given a plane index

  for (std::map<int, std::set<int>>::iterator it = clusters_.begin();
    it != clusters_.end(); it++) {
    bool ret = binary_search(it->second.begin(), it->second.end(), plane_index);
    if (ret) {
      cluster_result_p = it->first;
    }
  }
}


void GraspPointGenerator::eigen2PCL(
  const Eigen::Vector3d &eig,
  const Eigen::Vector3d &norm,
  pcl::PointXYZRGBNormal &pcl_point,
  int r, int g, int b) {
  // transform Eigen::Vector3d point with normal to pcl::PointXYZRGBNormal
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

void GraspPointGenerator::addPoint2Cloud(
  const Eigen::Vector3d &p,
  const Eigen::Vector3d &n_p,
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud) {
  // add Point to point cloud
  pcl::PointXYZRGBNormal pcl_point;
  eigen2PCL(p, n_p, pcl_point,
                    config_.point_color[0]*255,
                    config_.point_color[1]*255,
                    config_.point_color[2]*255);
  cloud->points.push_back(pcl_point);
  cloud->width++;
}

