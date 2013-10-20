#ifndef PLANE_SEGMENTOR_H
#define PLANE_SEGMENTOR_H

#pragma warning(disable : 4996)
#pragma warning(disable : 4267)
#pragma warning(disable : 4521)
#pragma warning(disable : 4244)
#pragma warning(disable : 4305)
#pragma warning(disable : 4503)

#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>


class PlaneSegmentor
{
public:
  PlaneSegmentor();
  ~PlaneSegmentor();

  void segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src,
    std::vector< pcl::PointCloud<pcl::PointXYZRGB> >& plane_clouds,
    std::vector< pcl::ModelCoefficients >& plane_coeffs);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getDisplayCloud() const;

private:
  pcl::ModelCoefficients::Ptr coefficients;
  pcl::PointIndices::Ptr inliers;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_display;
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  std::vector<pcl::PointIndices> cluster_indices;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> planeViewer;
};

#endif

