#include <iostream>
#include <thread>

#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

using namespace std;
using namespace std::chrono_literals ;


 pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
 {
   // --------------------------------------------
   // -----Open 3D viewer and add point cloud-----
   // --------------------------------------------
   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   viewer->addPointCloud<pcl::PointXYZI> (cloud, "sample cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
   viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
   return (viewer);
 }


 void octreeDownsampling(
    const PointCloudPtr &pointcloud,
    const PointCloudPtr &downsampled_pointcloud,
    float voxel_size)
{
    pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(voxel_size);
    octree.setInputCloud(pointcloud);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
    // Get PointT vector of centroids for all occupied voxels
    octree.getVoxelCentroids(centroids);

    // init current map with the downsampled full cloud
    downsampled_pointcloud->clear();
    downsampled_pointcloud->points.assign(centroids.begin(), centroids.end());
    downsampled_pointcloud->width = 1;
    downsampled_pointcloud->height = downsampled_pointcloud->points.size(); // make sure again the format of the downsampled point cloud
}

void octreeDownsamplingWithIntensity(
    const PointCloudPtr &pointcloud,
    const PointCloudPtr &downsampled_pointcloud,
    float voxel_size)
{
    PointCloudPtr pointcloud_ds(new PointCloud());
    pcl::PointCloud<PointType>::Ptr ptr_reassigned(new pcl::PointCloud<PointType>);

    pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(voxel_size);
    octree.setInputCloud(pointcloud);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
    // Get PointT vector of centroids for all occupied voxels
    octree.getVoxelCentroids(centroids);

    // init current map with the downsampled full cloud
    pointcloud_ds->clear();
    pointcloud_ds->points.assign(centroids.begin(), centroids.end());
    pointcloud_ds->width = 1;
    pointcloud_ds->height = pointcloud_ds->points.size(); // make sure again the format of the downsampled point cloud

    // static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    // voxel_filter.setInputCloud(pointcloud);
    // voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    // downsampled_pointcloud->clear();
    // voxel_filter.filter(*downsampled_pointcloud);

    if (true)
    {
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(pointcloud);

        ptr_reassigned->points.reserve(downsampled_pointcloud->points.size());

        int K = 1;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        // Set dst <- output
        for (const auto &pt : pointcloud_ds->points)
        {
            if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                auto updated = pt;
                // Update meaned intensity to original intensity
                updated.intensity = (*pointcloud)[pointIdxNKNSearch[0]].intensity;
                ptr_reassigned->points.emplace_back(updated);
            }
        }
        *downsampled_pointcloud = *ptr_reassigned;
        downsampled_pointcloud->width = 1;
        downsampled_pointcloud->height = downsampled_pointcloud->points.size();
    }
    else
    {
        *downsampled_pointcloud = *pointcloud_ds;
    }
}

int main(int argc, char** argv)
{
    std::ifstream myfile ("/home/tingxfan/rerun-4DMOS/exp_seq0.txt"); 

    vector<float> data;

    if ( myfile.is_open() ) { // always check whether the file is open
        while (myfile)
        {   
            float tem;

            myfile >> tem;
            data.push_back(tem);

            // std::cout << tem << std::endl; // pipe stream's content to standard output
        }
    }

    std::cout << "Done" << std::endl;
    std::cout << "data size: " << data.size() << std::endl;

// ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZI>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>& cloud = *cloud_ptr;

  std::cout << "Generating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  std::uint8_t r(255), g(15), b(15);
    
  for (int i = 0; i <= data.size(); i+=4)
  {
      pcl::PointXYZI basic_point;
      basic_point.x = data[i];
      basic_point.y = data[i+1];
      basic_point.z = data[i+2]+1.73;
      basic_point.intensity = data[i+3];
    //   std::cout << basic_point.intensity << std::endl;
      basic_cloud_ptr->points.push_back(basic_point);
  }


  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  octreeDownsampling(basic_cloud_ptr, cloud_filtered, 0.2);


  basic_cloud_ptr->width = basic_cloud_ptr->size ();
  basic_cloud_ptr->height = 1;

  cloud_filtered->width = cloud_filtered->size ();
  cloud_filtered->height = 1;


  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = simpleVis(cloud_filtered);

  pcl::io::savePCDFileBinary(string("exp_0.pcd"), *cloud_filtered);

    //--------------------
    // -----Main loop-----
    //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

}