#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataMapper.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/voxel_grid.h>

//typedef pcl::visualization::CloudViewer::MonochromeCloud PointCloud;


//void pointCloudToVoxelGrid(const PointCloud &inCloud, PointCloud &outCloud)
//{
//
//  const float voxel_grid_size = 0.05f; //0.005f;
//  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//  PointCloud::ConstPtr inCloudConstPtr(new PointCloud(inCloud));
//  vox_grid.setInputCloud (inCloudConstPtr);
//  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
//  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
//  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
//  vox_grid.filter (*tempCloud);
//  outCloud = *tempCloud;
//}

void generateViews(const std::string &modelFileName, const float resx, const float resy)
{
  typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName(modelFileName.c_str());
  reader->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());
  mapper->Update();

  vtkSmartPointer<vtkPolyData> object = mapper->GetInput();

  pcl::apps::RenderViewsTesselatedSphere render_views;
  render_views.addModelFromPolyData(object);
  render_views.generateViews(); 


//// Read in pcd file
//std::string filename = "/mnt/hgfs/llister/Devel/robomakery/data/woodenShelf/scene_with_shelf.pcd";
//pcl::io::loadPCDFile(filename, cloud);

//// Read in model file
//pcl::io::loadPLYFile(modelFileName, cloud);
//
//// Convert point cloud to vtk poly data
//vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

//pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudConstPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
//vtkSmartPointer<vtkPolyData> polydata;
//vtkSmartPointer<vtkIdTypeArray> initcells;
//vis.convertPointCloudToVTKPolyData<pcl::PointXYZ>(cloudConstPtr, polydata, initcells);
//pcl::visualization::convertPointCloudToVTKPolyData(cloudConstPtr, polydata, initcells);

  // Create const pointer (memcopy) for visualization
//PointCloud::ConstPtr cloudPtr(new PointCloud(cloud));

  // Downsample input point cloud to make a voxel grid
//PointCloud voxelGrid;
//pointCloudToVoxelGrid(cloud, voxelGrid);
//PointCloud::ConstPtr voxelGridPtr(new PointCloud(voxelGrid));

//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//viewer.showCloud(cloudPtr);
//viewer.showCloud(voxelGridPtr);

  // OLD WAY
//pcl::PointCloud<pcl::PointXYZ>::CloudVectorType views_xyz;
//std::vector <Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix<float, 4, 4> > > poses;
//std::vector<float> entropies;
//int tesselation_level = 1;
//vis.addModelFromPLYFile(modelFileName, "");
//vis.setRepresentationToSurfaceForAllActors ();
//vis.renderViewTesselatedSphere (resx, resy, views_xyz,
//                                poses, entropies, tesselation_level);

//pcl::apps::RenderViewsTesselatedSphere render_views;
//render_views.setResolution(resx);
//render_views.setTesselationLevel (1); //80 views
//render_views.addModelFromPolyData (mapper); //vtk model
//render_views.setGenOrganized(false);
//render_views.generateViews ();
//std::vector< CloudPtr > views;
//std::vector < Eigen::Matrix4f > poses;
//render_views.getViews (views);
//render_views.getPoses (poses);


//while (!viewer.wasStopped ())
//{
//}
}

int main(int argc, char **argv)
{
  if (argc != 3) {
    std::cerr << "Wrong # of input arguments" << std::endl;
    std::cerr << "USAGE: ./" << argv[0] << " MODEL_FILE.ply RESX" << std::endl;
    return -1;
  }

  std::string modelFileName = std::string(argv[1]);
  float resx = atof(argv[2]);
  float resy = resx;

  generateViews(modelFileName, resx, resy);

  return 0;
}
