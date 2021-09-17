#include "pgfat/grasp_point_generator.h"


void GraspPointGenerator::setConfig(const YAMLConfig &config) {
  config_ = config;
  CollisionCheck_.setConfig(config);
  CollisionCheck_.gripper_model_.setParams(
    config_.gripper_params[0],
    config_.gripper_params[1],
    config_.gripper_params[2],
    config_.gripper_params[3],
    config_.gripper_params[4],
    config_.gripper_params[5],
    config_.gripper_params[6],
    config_.gripper_params[7],
    config_.gripper_params[8],
    config_.gripper_params[9],
    config_.gripper_params[10],
    config_.gripper_params[11]);
}

void GraspPointGenerator::setMesh(
  const std::vector <TrianglePlaneData> &triangle_mesh,
  const std::vector <TrianglePlaneData> &finger_triangle_mesh) {
  planes_ = triangle_mesh;
  CollisionCheck_.loadMesh(triangle_mesh, finger_triangle_mesh);
}

void GraspPointGenerator::setClusters(
  const std::map<int, std::set<int>> &clusters) {
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

  // after sampling, make point pair for every point we sampled using makePair(n, p)

  std::default_random_engine generator;
  std::uniform_real_distribution<double> mesh_distribution(0.0, 1.0);

  for (std::map<int, std::set<int>>::iterator it = clusters_.begin();
    it != clusters_.end(); it++) {
    std::cout << "cluster now: " << it->first << "/"
              << clusters_.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ {new pcl::PointCloud<pcl::PointXYZRGBNormal>};
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
    if (totalArea <= 5)  continue; //diascard cluster which has too small total area
    
    random_point_num = totalArea;
    for (std::size_t i = 0; i < random_point_num; ++i) {
      Eigen::Vector3d p;
      Eigen::Vector3d n(0, 0, 0);
      double r = mesh_distribution(generator) * totalArea;
      double r1 = mesh_distribution(generator);
      double r2 = mesh_distribution(generator);
      randPSurface(planes_, Area_index, cumulativeAreas, totalArea, p, n, r, r1, r2);    
      addPoint2Cloud(p, n, config_.point_color, cloud_);
    }
 
    // remove sampled points too close to each other
    remove_close(cloud_, config_.Distance_rnn);
    // find the boudary of the cluster where grasp point lied on 
    // and use the boundaries to set the approach direction
    std::vector<edge> bdry;   
    findbdry(Area_index, bdry);    
    bdrys_.insert(std::pair<int, std::vector<edge>>(it->first, bdry));
       
    // find point pair for the points remaining
    for (size_t i = 0; i < cloud_->points.size (); ++i) {
      makePair(PCLNormal2eigen(cloud_->points[i]), PCL2eigen(cloud_->points[i]), bdry);
    }
  }
  
  std::sort(grasp_cand_collision_free1_.begin(),
            grasp_cand_collision_free1_.end(),
            [this](auto l, auto r) {return Areacompare(l, r); });
  std::sort(grasp_cand_collision_free2_.begin(),
            grasp_cand_collision_free2_.end(),
            [this](auto l, auto r) {return Areacompare(l, r); });
  
  std::cout << "all the sampled grasp points: "
            << candid_sample_cloud_->points.size()/2
            << std::endl;
  std::cout << "grasp points on the center which are collision free: "
            << grasp_cand_collision_free1_.size()
            << std::endl;
  std::cout << "grasp points on the finger surfaces which are collision free: "
            << grasp_cand_collision_free2_.size()
            << std::endl;
}


bool GraspPointGenerator::Areacompare(GraspData &g1, GraspData &g2) {
  return g1.getContactArea() > g2.getContactArea();
}
  
  
void GraspPointGenerator::makePair(
  const Eigen::Vector3d &n_p,
  const Eigen::Vector3d &p,
  const std::vector<edge> &bdry) {
  // firstly find the correspoding point for a given point to make a point pair
  // and then check if collide

  for (auto & plane : planes_) {
    auto & n_result_p = plane.normal;
    // opposite dir tolerance
    if ((n_p + n_result_p).norm() < config_.Theta_parl) {
      Eigen::Vector3d result_p;
      calcLinePlaneIntersection(plane, p, -n_p, result_p);

      if (!pointInTriangle(result_p, plane))
        continue;
      if (!strokeCollisionCheck(p, result_p))
        continue;
        
      std::vector<Eigen::Vector3d> Dir;
      setApproachDir(p, n_p, bdry, Dir);

      for (auto & dir : Dir) {
        // add this point pair to sampled point cloud
        addPoint2Cloud(p, n_p, config_.point_color, candid_sample_cloud_);
        addPoint2Cloud(result_p, n_result_p, config_.point_color, candid_sample_cloud_);

        // create graspdata
        GraspData gd;
        gd.hand_transform.linear().col(0) = dir;  // X
        gd.hand_transform.linear().col(1) = n_p.cross(dir);  // Y = Z cross X
        gd.hand_transform.linear().col(2) = n_p;  // Z
        gd.hand_transform.translation() = (p + result_p) / 2;  // tip position
        gd.points.push_back(p);
        gd.points.push_back(result_p);
        grasps_.push_back(gd);

        // check collision for every graspdata
        int mode = 1;
        if (!collisionCheck(gd, mode)) {
          // after collision check add this point pair to result point cloud
          addPoint2Cloud(gd.points[0],
                         gd.hand_transform.linear().col(2),
                         config_.point_color,
                         candid_result_cloud1_);
          addPoint2Cloud(gd.points[1],
                        -gd.hand_transform.linear().col(2),
                         config_.point_color,
                         candid_result_cloud1_);
          // add collision free graspdata to data set
          grasp_cand_collision_free1_.push_back(gd);
        } else {
          grasp_cand_in_collision_.push_back(gd);
        }
        mode = 2;
        if (!collisionCheck(gd, mode)) {
          // after collision check add this point pair to result point cloud
          addPoint2Cloud(gd.points[0],
                         gd.hand_transform.linear().col(2),
                         config_.point_color,
                         candid_result_cloud2_);
          addPoint2Cloud(gd.points[1],
                        -gd.hand_transform.linear().col(2),
                         config_.point_color,
                         candid_result_cloud2_);
          // add collision free graspdata to data set
          grasp_cand_collision_free2_.push_back(gd);
        } else {
          grasp_cand_in_collision_.push_back(gd);
        }
      }
    }
  }
}


bool GraspPointGenerator::collisionCheck(GraspData &grasp, const int &mode) {
  if (mode == 1) {
    // contact position in center   
    if (grasp.getDist()/2 > 2 * config_.gripper_params[9]) {
      // distance between two points must be larger than minimum     
      double totalCost = CollisionCheck_.isCollide(grasp.hand_transform, mode,
                                                   grasp.getDist()/2.0 - config_.gripper_params[9] + 0.001);
      if (totalCost == 0) {
        grasp.contactArea = 0;
        return true;
      } else {
        grasp.contactArea = totalCost;
        return false;
      }
    } 
  }
  
  if (mode == 2) {   
    // contact position in 2 finger pads
    double totalCost = CollisionCheck_.isCollide(grasp.hand_transform, mode,
                                                 grasp.getDist()/2.0 + 0.001);
    if (totalCost == 0) {
      grasp.contactArea = 0;
      return true;
    } else {
      grasp.contactArea = totalCost;
      return false;
    }
  }
}


void GraspPointGenerator::findbdry(std::vector<int> & Area_index, std::vector<edge> &bdry) {
  // find the border edges of a set of triangles
  // Area_index: input adjacent triangles
  // bdry: border edges
  
  std::vector<edge> visited_edge; 
  std::vector<int> nums (Area_index.size() * 3);  
  int i = 0;  
  
  for (auto & index : Area_index) {  
    int j = 0;   
    for (auto & ed : planes_[index].edges) {     
      nums[i*3+j] = 0;
      int k = 0;      
      for (auto & vi : visited_edge) {        
        if ((abs((vi.first - ed.first).norm()) == 0.0)&&(abs((vi.second - ed.second).norm()) == 0.0) 
         ||((abs((vi.first - ed.second).norm()) == 0.0)&&(abs((vi.second - ed.first).norm()) == 0.0))) {
          nums[k] += 1;
          nums[i*3+j] += 1; 
        }
        ++k;
      }
      visited_edge.push_back(ed);
      ++j;       
    }
    ++i;
  }
  
  for (int in = 0; in <nums.size(); in++) {
    std::vector<edge>::iterator vit = visited_edge.begin();
    if (nums[in] == 0) {      
      vit+=in;
      bdry.push_back(*vit);
    }
  }   
}


void GraspPointGenerator::setApproachDir(
  const Eigen::Vector3d &p,
  const Eigen::Vector3d &n_p,
  const std::vector<edge> &bdry,
  std::vector<Eigen::Vector3d> &Dir) {
  // set possible approach directions with a given grasp point
  // p and n_p: position and normal direction of grasp point
  // bdry: border edges of a set of triangles where p lies
  // Dir: the found approach directions

  bool issame1 = false;
  bool issame2 = false;
  if (config_.Approach_boundary == true) {
    // set approach direction based on rotation based on the border edges
    for (auto & bdry : bdry) {
      Eigen::Vector3d e = (bdry.first - bdry.second).normalized();
      Eigen::Vector3d approach_direction = n_p.cross(e);
      
      for (auto & dir : Dir) {         
        if ((dir - approach_direction).norm() <= 0.01) {          
          issame1 = true;
          break;
        }
      }
      for (auto & dir : Dir) {
        if ((dir + approach_direction).norm() <= 0.01) {          
          issame2 = true;
          break;
        }
      }
      if (!issame1) {
        Dir.push_back(approach_direction);
      }
      if (!issame2) {
        Dir.push_back(-approach_direction);       
      }
      issame1 = false; 
      issame2 = false;                
    }
   
  }
  else if (config_.Approach_rotation == true) {
    // set approach direction based on rotation
    
    //Eigen::Vector3d axis = (p - result_p).normalized();
    Eigen::Vector3d axis = n_p;
    Eigen::Vector3d orth = getOrthogonalVector(axis);
    double theta;
    for (int i = 0; i < config_.N_da; ++i) {
      theta = (360.0 / double(config_.N_da)) * i;
      if (theta > 180.0) {
        theta = theta - 360.0;               
      }
      theta = (theta * M_PI) / 180.0;
      Eigen::Vector3d approach_direction = Eigen::AngleAxisd(theta, axis).matrix() * orth;
      Dir.push_back(approach_direction);
    }
  }
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
    
     
void GraspPointGenerator::remove_close(
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_,
  const double &radius) {
  // Neighbors within radius search
  // ensures that distance between any two points in point cloud does not exceed radius
    
  pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
  kdtree.setInputCloud(cloud_);  
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;  
  std::set<int> temp;

  for (std::size_t i = 0; i < cloud_->points.size(); ++i) {    
    std::set<int>::iterator pos = temp.find(i);
    if (pos != temp.end())
      continue;      
    pcl::PointXYZRGBNormal searchPoint = cloud_->points[i];  
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    
    if (kdtree.radiusSearch(searchPoint, radius, 
        pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1 ) {
      for (std::size_t j = 1; j < pointIdxRadiusSearch.size(); ++j) {
        temp.insert(pointIdxRadiusSearch[j]);       
      } 
    }    
  }  
  for (std::set<int>::iterator it = temp.begin(); it != temp.end(); it++) {
    inliers->indices.push_back(*it);
  }
  //std::cout<<"size of cloud "<<cloud_->points.size()<<std::endl;      
  extract.setInputCloud(cloud_);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_);
  //std::cout<<"size of indices "<<inliers->indices.size()<<std::endl;
  //std::cout<<"size of cloud now "<<cloud_->points.size()<<std::endl;
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










































