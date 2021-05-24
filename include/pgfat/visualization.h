

#pragma once
#include <iostream>
#include <random>
#include <string>

#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataWriter.h>
#include <vtkSmartPointer.h>
#include <vtkVertex.h>
#include "vtkTriangle.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"


#include "pgfat/geometrics.h"
#include "pgfat/mesh_sampling.h"
#include "pgfat/triangle_plane_data.h"
#include "pgfat/yaml_config.h"



class Visualizer
{
public:

  void setConfig(const YAMLConfig &config);
  void setMesh(const std::vector <TrianglePlaneData>& triangle_mesh);
  void display_initial(const pcl::PolygonMesh& mesh);
  void display_reconstruct(const std::vector <TrianglePlaneData>& triangles);
  void display_cluster(const std::vector <TrianglePlaneData>& triangles,const std::set<std::set<int>>& clusters);
  
 
private:

  std::vector <TrianglePlaneData> planes_;
  YAMLConfig config_;

};
