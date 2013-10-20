#include "PlaneSegmentor.h"
#include <iostream>


const int numColors = 20;
const int gColors[numColors][3] = {
  {255, 0, 0},
  {0, 255, 0},
  {0, 0, 255},
  {255, 255, 0},
  {0, 255, 255},
  {255, 0, 255},
  {255, 165, 0},
  {255, 20, 147},
  {160, 32, 240},
  {255, 218, 185},
  {190, 190, 190},
  {100, 149, 237},
  {0, 191, 255},
  {143, 188, 143},
  {124, 252, 0},
  {184, 134, 11},
  {205, 92, 92},
  {250, 128, 114},
  {255, 69, 0},
  {221, 160, 221}
};


//#define DISPLAY_POINT_CLOUD
#undef DISPLAY_POINT_CLOUD


//------------------------------------------------------------
PlaneSegmentor::PlaneSegmentor()
  : coefficients(new pcl::ModelCoefficients),
  inliers(new pcl::PointIndices()),
  cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>),
  cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>),
  cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>),
  cloud_display(new pcl::PointCloud<pcl::PointXYZRGB>),
  tree(new pcl::search::KdTree<pcl::PointXYZRGB>)
#ifdef DISPLAY_POINT_CLOUD
  , planeViewer(new pcl::visualization::PCLVisualizer ("3D Planes"))
#endif
{
  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setDistanceThreshold (0.01);
  seg.setDistanceThreshold (0.05);
  seg.setMaxIterations (1000);

  ec.setClusterTolerance (0.5); 
  ec.setMinClusterSize (450);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
}

//------------------------------------------------------------
PlaneSegmentor::~PlaneSegmentor()
{
}

//------------------------------------------------------------
void PlaneSegmentor::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src,
  std::vector< pcl::PointCloud<pcl::PointXYZRGB> >& plane_clouds,
  std::vector< pcl::ModelCoefficients >& plane_coeffs)
{
  plane_clouds.clear();
  plane_coeffs.clear();

  // Store a copy of the source point cloud, to retain the original data
#if 0
  *cloud_filtered = *src;
#else
  sor.setInputCloud (src);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cout << "Reduced " << src->points.size() << " down to " << cloud_filtered->points.size() << std::endl;
#endif

  int ii = 0, colorIdx = 0;
  int nr_points = (int) cloud_filtered->points.size ();

  cloud_display->clear();
  cloud_display->reserve(nr_points);

#ifdef DISPLAY_POINT_CLOUD
  char shapeStr[1024];
  planeViewer->removeAllShapes();
#endif

  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered->makeShared());
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

#ifdef DISPLAY_POINT_CLOUD
    sprintf(shapeStr, "plane%04d", ii+1);
    planeViewer->addPlane(*coefficients, shapeStr);
#endif

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    const int npts = static_cast<int>(cloud_p->points.size());
    std::cerr << "PointCloud representing the planar component: " << npts << " data points." << std::endl;

    // Select color based on plane index
    int red = gColors[colorIdx][0];
    int green = gColors[colorIdx][1];
    int blue = gColors[colorIdx][2];
    ++colorIdx;
    if (colorIdx >= numColors)
    {
      colorIdx -= numColors;
    }

    plane_coeffs.push_back( *coefficients );
    plane_clouds.push_back( pcl::PointCloud<pcl::PointXYZRGB>() );
    pcl::PointCloud<pcl::PointXYZRGB>& newCloud = plane_clouds.back();
    newCloud.points.resize(cloud_p->points.size());
    std::copy(cloud_p->points.begin(), cloud_p->points.end(), newCloud.points.begin());
    newCloud.height = 1;
    newCloud.width = (uint32_t)newCloud.points.size();

    for (int ll = 0; ll < npts; ll++)
    {
      pcl::PointXYZRGB pt(red, green, blue);
      pt.x = cloud_p->points[ll].x;
      pt.y = cloud_p->points[ll].y;
      pt.z = cloud_p->points[ll].z;

      cloud_display->push_back(pt);
    }

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    ii++;
  }


  //// Segment objects from non-planar points
  //if (cloud_filtered->points.size() > ec.getMinClusterSize())
  //{
  //  tree->setInputCloud (cloud_filtered->makeShared());
  //  ec.setInputCloud (cloud_filtered->makeShared());
  //  ec.extract (cluster_indices);

  //  int j = 0;
  //  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  //  {
  //    // Select color based on plane index
  //    int red = gColors[colorIdx][0];
  //    int green = gColors[colorIdx][1];
  //    int blue = gColors[colorIdx][2];
  //    ++colorIdx;
  //    if (colorIdx >= numColors)
  //    {
  //      colorIdx -= numColors;
  //    }

  //    const int npts = (int)it->indices.size();
  //    std::cerr << "PointCloud representing the object component: " << npts << " data points." << std::endl;
  //    if (npts == 0)
  //    {
  //      continue;
  //    }

  //    // Store points into display cloud
  //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
  //    {
  //      pcl::PointXYZRGB pt;
  //      pt.x = cloud_filtered->points[*pit].x;
  //      pt.y = cloud_filtered->points[*pit].y;
  //      pt.z = cloud_filtered->points[*pit].z;

  //      //cloud_display->push_back(pt);
  //      objCloud->push_back(pt);
  //    }

  //    //sprintf(shapeStr, "object%04d", j+1);
  //    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(objCloud, red, green, blue);
  //    //planeViewer->addPointCloud<pcl::PointXYZ>(objCloud, shapeStr);

  //    j++;
  //  }
  //}

#ifdef DISPLAY_POINT_CLOUD
  if (!planeViewer->wasStopped())
  {
    planeViewer->spinOnce();
  }
#endif
}

//------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlaneSegmentor::getDisplayCloud() const
{
  return cloud_display;
}

