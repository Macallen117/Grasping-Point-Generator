

#pragma once
#include <vector>
#include <list>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkNew.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCell.h>
#include <vtkDataSetMapper.h>
#include <vtkExtractSelection.h>
#include <vtkIdList.h>
#include <vtkIdTypeArray.h>
#include <vtkNamedColors.h>
#include <vtkPolyData.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/features/integral_image_normal.h>

#include "pgfat/data_structure.h"

static std::vector<TrianglePlaneData> buildTriangleData(pcl::PolygonMesh & mesh) {
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(mesh, polydata);

  vtkSmartPointer<vtkTriangleFilter> triangleFilter
    = vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputData(polydata);
  triangleFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper
    = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
  triangleMapper->Update();
  polydata = triangleMapper->GetInput();

  polydata->BuildCells();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

  double p1[3], p2[3], p3[3];
  std::vector<TrianglePlaneData> triangles;

  vtkIdType npts = 0;
  vtkIdType *ptIds = nullptr;
  vtkIdType cellId = 0;

  for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds);) {
    TrianglePlaneData plane_data;
    polydata->GetPoint(ptIds[0], p1);
    polydata->GetPoint(ptIds[1], p2);
    polydata->GetPoint(ptIds[2], p3);
    plane_data.area = vtkTriangle::TriangleArea(p1, p2, p3);

    std::list<vtkIdType> neighbors;
    auto cellPointIds = vtkSmartPointer<vtkIdList>::New();
    polydata->GetCellPoints(cellId, cellPointIds);

    // triangleFilter->GetOutput()->GetCellPoints(cellId, cellPointIds);

    for (vtkIdType i = 0; i < cellPointIds->GetNumberOfIds(); i++) {
      auto idList = vtkSmartPointer<vtkIdList>::New();
      // add one of the edge points
      idList->InsertNextId(cellPointIds->GetId(i));
      // add the other edge point
      if (i + 1 == cellPointIds->GetNumberOfIds()) {
        idList->InsertNextId(cellPointIds->GetId(0));
      } else {
        idList->InsertNextId(cellPointIds->GetId(i + 1));
      }
      // get the neighbors of the cell
      auto neighborCellIds = vtkSmartPointer<vtkIdList>::New();
      polydata->GetCellNeighbors(cellId, idList, neighborCellIds);
      // triangleFilter->GetOutput()->GetCellNeighbors(cellId, idList, neighborCellIds);

      for (vtkIdType j = 0; j < neighborCellIds->GetNumberOfIds(); j++) {
        neighbors.push_back(neighborCellIds->GetId(j));
        plane_data.neighbors.insert(neighborCellIds->GetId(j));
      }
      /*
      std::cout << "cell id is: " << cellId << std::endl;
      std::cout << "Edge neighbor ids are: " << std::endl;
       for (auto it1 = plane_data.neighbors.begin(); it1 != plane_data.neighbors.end(); ++it1)
       {
         std::cout << " " << *it1;
       }
       std::cout << std::endl;
       */     
    }
    cellId++;
    Eigen::Vector3d v1 = Eigen::Vector3d(p1[0], p1[1], p1[2])
                       - Eigen::Vector3d(p3[0], p3[1], p3[2]);
    Eigen::Vector3d v2 = Eigen::Vector3d(p2[0], p2[1], p2[2])
                       - Eigen::Vector3d(p3[0], p3[1], p3[2]);
    Eigen::Vector3d v3 = Eigen::Vector3d(p1[0], p1[1], p1[2])
                       - Eigen::Vector3d(p2[0], p2[1], p2[2]);
    Eigen::Vector3d n = v1.cross(v2);
    n.normalize();
    // std::cout << "n: " << n.transpose() << std::endl;
        
    plane_data.points.resize(3);
    plane_data.points[0] = (Eigen::Map<const Eigen::Vector3d>(p1));
    plane_data.points[1] = (Eigen::Map<const Eigen::Vector3d>(p2));
    plane_data.points[2] = (Eigen::Map<const Eigen::Vector3d>(p3));
    plane_data.normal = n;
    plane_data.centroid = (plane_data.points[0] 
    			  + plane_data.points[1] 
    			  + plane_data.points[2]) / 3;    
    plane_data.edges[0] = (std::make_pair(plane_data.points[0], plane_data.points[1]));
    plane_data.edges[1] = (std::make_pair(plane_data.points[1], plane_data.points[2]));
    plane_data.edges[2] = (std::make_pair(plane_data.points[0], plane_data.points[2]));
    
    double a = (plane_data.edges[0].first - plane_data.edges[0].second).norm();
    double b = (plane_data.edges[1].first - plane_data.edges[1].second).norm();
    double c = (plane_data.edges[2].first - plane_data.edges[2].second).norm();
    
    double r = a + b + c;
    for(int i=0; i<3; ++i)
    {
      plane_data.incenter(i) = (a * plane_data.points[2](i) 
                              + b * plane_data.points[1](i) 
                              + c * plane_data.points[0](i)) / r;
    }
    triangles.push_back(plane_data);
  }

  return triangles;
}
