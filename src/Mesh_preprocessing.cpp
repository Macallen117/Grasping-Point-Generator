#include "pgfat/Mesh_preprocessing.h"
#define PI 3.14159265


void Mesh_preprocessor::setConfig(const YAMLConfig &config)
{
  config_ = config;
}

void Mesh_preprocessor::Print_Triangles(const std::vector <TrianglePlaneData> &triangles)
{
  for (auto it = triangles.begin(); it != triangles.end(); it++)
  {
    cout << "Normal:" << (*it).normal << endl;
    cout <<" point1:" << (*it).points[0] << endl;
    cout <<" point2:" << (*it).points[1] << endl;
    cout <<" point3:" << (*it).points[2] << endl;
  }

}

void Mesh_preprocessor::RegionGrow(const std::vector <TrianglePlaneData> &triangles)
{
  //Point cloud based region-growing method
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
   
  cloud->points.resize(triangles.size()*3);
 
  int count = 0;
  for (auto it = triangles.begin(); it != triangles.end(); it++)
  {
    for (int i = 0; i < 3; i++)
    {
      cloud->points[count].x = (*it).points[i](0);
      cloud->points[count].y = (*it).points[i](1);
      cloud->points[count].z = (*it).points[i](2);
    }
  }


  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (10);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  
  int counter = 0;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0) std::cout << std::endl;
  }
  std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }
}

bool Mesh_preprocessor::CheckNormal(const Eigen::Vector3d& Points1,const Eigen::Vector3d& Points2)
{
  double dot_product, result;
  dot_product = Points1(0) * Points2(0) + Points1(1) * Points2(1) + Points1(2) * Points2(2);
  result = acos (dot_product) * 180.0 / PI;
  //std::cout<<"The degrees:"<<result<<std::endl;
  if (result < config_.Theta1) return true;
  else return false;
}


std::set<int> Mesh_preprocessor::find_neibour(const std::vector <TrianglePlaneData> &triangles, const int& seed_index)
{	
  //find neibouring Triangle of the seed triangle       
  //The normals of the founded Triangles should be coherent with the seed
	
  // nIndex: number of triangles
  // open_set: set for expansion
  // closed_set: expanded index will be added
  // selected_set: set of index which fulfills threshold theta1
  // return: neighbour_set: a set of neighbouring triangles
			
  std::set<int>neighbour_set;
  std::set<int>open_set;
  std::set<int>closed_set;
  std::set<int>selected_set;
  std::set<int>difference_set;
  std::set<int>surrounding_set;
  Eigen::Vector3d seed_triangle_normal;
  int expand_id;
  int nIndex = triangles.size();	
  open_set.insert(seed_index); 
  seed_triangle_normal = triangles[seed_index].normal;

  while (open_set.size() != 0)
  {
    expand_id = *open_set.begin(); 
    selected_set.clear();
    difference_set.clear();
    surrounding_set.clear();
    
    /*		
    for(int i = 0; i < nIndex; i++)
    {
      for(int j = 0; j < 3; j++)
      {
        if((triangles[i].points[0] == triangles[expand_id].points[j])||(triangles[i].points[1] == triangles[expand_id].points[j])||(triangles[i].points[2] == triangles[expand_id].points[j]))  // todo: reduct computational time
        {
          surrounding_set.insert(i);   //find neighbours based on three vertex of seed 
        }
      }        	
    }
    */ 
    surrounding_set.insert(triangles[expand_id].neighbors.begin(),triangles[expand_id].neighbors.end());   
    //surrounding_set.erase(expand_id);
                    
    for(std::set<int>::iterator it = surrounding_set.begin(); it != surrounding_set.end(); it++)
    {
      if(CheckNormal(triangles[*it].normal,seed_triangle_normal))    selected_set.insert(*it); 
    }
                       
    neighbour_set.insert(selected_set.begin(),selected_set.end()); //all face id fulfillstheta1                     
    closed_set.insert(expand_id);        		
    open_set.erase(expand_id);         		
    set_difference( selected_set.begin(), selected_set.end(),closed_set.begin(), closed_set.end(),inserter( difference_set, difference_set.begin() ) ); 
    open_set.insert(difference_set.begin(),difference_set.end()); 
  }	
  return neighbour_set;  
}


std::set<std::set<int>> Mesh_preprocessor::RegionGrowing(const std::vector <TrianglePlaneData> &triangles)
{           
  //RegionGrowing method used to do preprocessing to mesh model
       
  // nIndex = number of triangles
  // all_triangles_index_set: set for all the triangles
  // seg_triangles_index_set: segmented triangle set
  // return: clusters: every set in clusters is a cluster of triangles for a given seed triangle

       
  int nIndex = triangles.size();
  int seed_index;
  std::vector < Eigen::Vector3d > seed_triangle {3};
  std::set<int>all_triangles_index_set; 
  std::set<int>neibour_triangle_index_set;
  std::set<int>seg_triangles_index_set;
  std::set<int>seg_seed_index_set; 
  std::set<int>index_difference_set; 
  std::set<std::set<int>> clusters; 
	
  for (int i = 0; i < nIndex; i++)
  {
    all_triangles_index_set.insert(i);  
  }
						
  while(all_triangles_index_set.size() != 0)  
  {
    seed_index = *all_triangles_index_set.begin(); 		
    neibour_triangle_index_set = find_neibour(triangles, seed_index); 
    neibour_triangle_index_set.insert(seed_index);  //one cluster
				
    set_difference(neibour_triangle_index_set.begin(), neibour_triangle_index_set.end(),seg_triangles_index_set.begin(), seg_triangles_index_set.end(),inserter( index_difference_set, index_difference_set.begin() ) ); 		
    seg_triangles_index_set.insert(index_difference_set.begin(),index_difference_set.end());
    for (std::set<int>::iterator it = index_difference_set.begin(); it != index_difference_set.end(); it++)
    {
      all_triangles_index_set.erase(*it);
    }
    std::cout<<"all size:"<<all_triangles_index_set.size()<<std::endl;
    std::cout<<"seg size:"<<seg_triangles_index_set.size()<<std::endl;		
    clusters.insert(neibour_triangle_index_set);
				
    index_difference_set.clear();
    neibour_triangle_index_set.clear();				
  }
  return clusters;
}



