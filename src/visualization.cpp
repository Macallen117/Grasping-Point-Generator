#include "pgfat/visualization.h"


void Visualizer::setConfig(const YAMLConfig &config)
{
  	config_ = config;
}

void Visualizer::setMesh(const std::vector <TrianglePlaneData> triangle_mesh)
{
  	planes_ = triangle_mesh;
	
}

void Visualizer::display_initial(pcl::PolygonMesh& mesh)
{ 
   pcl::visualization::PCLVisualizer vis1 ("Initial Point clouds");
   //vis1.addPointCloud<pcl::PointXYZRGBNormal> (candid_sample_cloud_);
   vis1.addPolygonMesh(mesh, "meshes",0);
   vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, config_.mesh_color[0], config_.mesh_color[1], config_.mesh_color[2], "meshes");
   vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "meshes");
   vis1.setBackgroundColor (config_.background_color[0], config_.background_color[1], config_.background_color[2]);
   vis1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, config_.point_size);
   
   vis1.spin ();
}

// viz.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "mesh_id");

