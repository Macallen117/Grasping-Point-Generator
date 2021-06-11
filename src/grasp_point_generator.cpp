#include "pgfat/grasp_point_generator.h"


void GraspPointGenerator::setConfig(const YAMLConfig &config)
{
  config_ = config;
}

void GraspPointGenerator::setMesh(const std::vector <TrianglePlaneData> &triangle_mesh)
{
  planes_ = triangle_mesh;	
}

void GraspPointGenerator::setClusters(const std::set<std::set<int>> &clusters)
{
  clusters_ = clusters;
}
void GraspPointGenerator::randomPointGenerate()
{
  setSampleMethode();
}

void GraspPointGenerator::setSampleMethode()
{
  if (config_.point_generation_method == "random_sample")
  {
    randomSample();
  }
}

void GraspPointGenerator::randomSample ()
{
  std::default_random_engine generator;
  std::uniform_real_distribution<double> mesh_distribution(0.0,1.0);  

  for (std::set<std::set<int>>::iterator it = clusters_.begin(); it != clusters_.end(); it++) 
  {
    if (  candid_sample_cloud_->points.size() !=0) break;
    double totalArea = 0;    
    int random_point_num;
    std::vector<double> cumulativeAreas;
    std::vector<int> Area_index;
    
    for (std::set<int>::iterator vit = (*it).begin(); vit != (*it).end(); vit++) 
    {
      totalArea += planes_[*vit].area;
      cumulativeAreas.push_back(totalArea);
      Area_index.push_back(*vit);            
    }   
    random_point_num = totalArea;  
    for (std::size_t i = 0; i < random_point_num; i++)
    {
      Eigen::Vector3d p;  
      Eigen::Vector3d n (0, 0, 0);  
      double r = mesh_distribution(generator) * totalArea;      //r :(0-1)*totalArea
      double r1 = mesh_distribution(generator);  // r1, r2: 0-1
      double r2 = mesh_distribution(generator);
      randPSurface (planes_, Area_index, cumulativeAreas, totalArea, p, n, r, r1, r2); //return p,n
      makePair(n, p);
      //std::cout<<"size:"<<candid_sample_cloud_->points.size()<<std::endl; 
      //std::cout<<"size:"<<candid_result_cloud_->points.size()<<std::endl;
      if (  candid_sample_cloud_->points.size() !=0) break;   
    }     
  }     
}

void GraspPointGenerator::makePair(const Eigen::Vector3d &n_p, const Eigen::Vector3d &p)
{
  pcl::PointXYZRGBNormal pcl_point_1;
  pcl::PointXYZRGBNormal pcl_point_2;
  for (auto & plane : planes_)
  {
    auto & n_result_p = plane.normal;
    if ((n_p+n_result_p).norm() < config_.Theta_parl) // opposite dir tolerance
    {
      Eigen::Vector3d result_p;
      Eigen::Vector3d p_2;
      Eigen::Vector3d result_p_2;
      calcLinePlaneIntersection(plane, p, -n_p, result_p);

      if (!pointInTriangle(result_p, plane)) continue;          
      if (!strokeCollisionCheck(p, result_p)) continue;   
     
      Eigen::Vector3d axis = (p - result_p).normalized();
      double theta;
      Eigen::Matrix3d rotmax;
      for(int i = 0; i < config_.N_da; i++)
      {
        theta = 360.0 / config_.N_da * i; 
        rotmax = Eigen::AngleAxisd(theta, axis).matrix();
        if (!setSecondFinger(p, n_p, result_p, rotmax, p_2, result_p_2)) continue;      
        //if (!handCollisionCheck()) continue;   
        //add point pair
     
        eigen2PCL(p, n_p*15, pcl_point_1, config_.point_color[0]*255,config_.point_color[1]*255,config_.point_color[2]*255); //n times 15 for better visualization
        candid_sample_cloud_->points.push_back(pcl_point_1);
        candid_sample_cloud_->width++;
        
        eigen2PCL(p_2, n_p*15, pcl_point_1, config_.point_color[0]*255,config_.point_color[1]*255,config_.point_color[2]*255);
        candid_sample_cloud_->points.push_back(pcl_point_1);
        candid_sample_cloud_->width++;
       
                 
        eigen2PCL(result_p, n_result_p*15, pcl_point_2, config_.point_color[0]*255,config_.point_color[1]*255,config_.point_color[2]*255); 
        candid_result_cloud_->points.push_back(pcl_point_2);
        candid_result_cloud_->width++;
        
        eigen2PCL(result_p_2, n_result_p*15, pcl_point_2, config_.point_color[0]*255,config_.point_color[1]*255,config_.point_color[2]*255); 
        candid_result_cloud_->points.push_back(pcl_point_2);
        candid_result_cloud_->width++;
        //add graspdata
        
        //grasp data
      }     
    }
  }
}

bool GraspPointGenerator::strokeCollisionCheck(const Eigen::Vector3d &p, const Eigen::Vector3d &result_p)
{
  auto dist = result_p - p;
  if (abs(dist.norm()) < config_.Stroke_per_jaw * 2) return true;  
  else return false;
}

bool GraspPointGenerator::setSecondFinger(const Eigen::Vector3d &p, const Eigen::Vector3d &n_p, const Eigen::Vector3d &result_p, const Eigen::Matrix3d &rotmax, Eigen::Vector3d &p_2, Eigen::Vector3d &result_p_2)
{  
  Eigen::Vector3d orth = getOrthogonalVector(n_p);  
  Eigen::Vector3d dir = rotmax * orth;     //dir for calculating the second point
  p_2 = p + config_.gripper_fingers_dist * dir;
  result_p_2 = result_p + config_.gripper_fingers_dist * dir;
  
  if ((pointInObject(p_2, planes_))&&(pointInObject(result_p_2, planes_)))  return true;
  return false;
}


void GraspPointGenerator::eigen2PCL(const Eigen::Vector3d &eig, const Eigen::Vector3d &norm, pcl::PointXYZRGBNormal &pcl, int r, int g, int b)
{
  pcl.x = eig(0);
  pcl.y = eig(1);
  pcl.z = eig(2);
  pcl.normal_x = norm(0);
  pcl.normal_y = norm(1);
  pcl.normal_z = norm(2);
  pcl.r = r;
  pcl.g = g;
  pcl.b = b;
}



