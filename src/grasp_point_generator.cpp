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
    random_point_num = totalArea * 20;  
    for (std::size_t i = 0; i < random_point_num; i++)
    {
      Eigen::Vector3d p;  
      Eigen::Vector3d n (0, 0, 0);  
      double r = mesh_distribution(generator) * totalArea;      //r :(0-1)*totalArea
      double r1 = mesh_distribution(generator);  // r1, r2: 0-1
      double r2 = mesh_distribution(generator);
      randPSurface (planes_, Area_index, cumulativeAreas, totalArea, p, n, r, r1, r2); //return p,n
      pcl::PointXYZRGBNormal pcl_point_1;
      eigen2PCL(p, n, pcl_point_1, config_.point_color[0]*255,config_.point_color[1]*255,config_.point_color[2]*255);
      candid_sample_cloud_->points.push_back(pcl_point_1);  //todo: discard bad sample if necessary
    }    
  }
  /*   
  std::cout<<"candid_sample_cloud_ data: "<<candid_sample_cloud_->points.size()<<"points"<<std::endl;
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
  outrem.setInputCloud(candid_sample_cloud_);              
  outrem.setRadiusSearch(0.5);              
  //outrem.setMinNeighborsInRadius(1);       
  outrem.filter (*candid_result_cloud);
  std::cout<<"candid_result_cloud data: "<<candid_result_cloud->points.size()<<"points"<<std::endl;
  */
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



