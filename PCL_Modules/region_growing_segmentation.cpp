#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/crop_box.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr getGroundCluster (std::vector <pcl::PointIndices>, pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr getGroundCluster (std::vector <pcl::PointIndices> given_cluster, pcl::PointCloud<pcl::PointXYZ>::Ptr given_cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_ground (new pcl::PointCloud<pcl::PointXYZRGB>);


  // copyPointCloud(test_cloud, given_cloud);
  int max_size = 0;
  int max_int = 0;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  if (!given_cluster.empty ()){
    // std::vector<unsigned char> colors;
    for (int i=0; i<given_cluster.size(); i++){ 
     std::cout << "Cluster " << i << " has "<< given_cluster[i].indices.size () << " points." << std::endl;
      if (max_size < given_cluster[i].indices.size()){
        max_size = given_cluster[i].indices.size();
        max_int = i;
    }


    // for (const auto& i_point: given_cluster)
    // {
    //   pcl::PointXYZRGB point;
    //   point.x = *(i_point.data);
    //   point.y = *(i_point.data + 1);
    //   point.z = *(i_point.data + 2);
    //   point.r = 255;
    //   point.g = 0;
    //   point.b = 0;
    //   test_colored_cloud->points.push_back (point);
    // }

    

    
  }

  }

  pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(given_cluster[max_int]));
  extract.setInputCloud (given_cloud);
  extract.setIndices (cluster_ptr);
  extract.setNegative (false);
  extract.filter (*test_cloud);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (test_cloud);
  seg.segment (*inliers, *coefficients);

  // We are trying this
  // Eigen::VectorXf model_coefficients = static_cast(Eigen::VectorXf, *coefficients);

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                << coefficients->values[1] << " "
                                << coefficients->values[2] << " " 
                                << coefficients->values[3] << std::endl;

  std::cout << "Ground plane Cluster has "<< max_size << " points and index " <<  max_int << std::endl;

  return test_cloud;
};


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("../scene2aa_ouster2.pcd", *cloud) == -1)
  // if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("../new_ouster2.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }


// Removing the points of the Truck since they do not help in any form of overlapping
  pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropBox<pcl::PointXYZ> boxFilter(true); // argumaent true can be ignored as well in our case
  boxFilter.setMin(Eigen::Vector4f(-3.5, -3.5, -3.5, 1.0));
  boxFilter.setMax(Eigen::Vector4f(3.5, 3.5, 3.5, 1.0));
  boxFilter.setInputCloud(cloud);
  boxFilter.setNegative(true);
  boxFilter.filter(*bodyFiltered); 
  std::vector<int> removedIndices =  *(boxFilter.getRemovedIndices());
  std::vector<int> actualindices = *(boxFilter.getIndices());
  // boxFilter.applyFilter(*indices2);


  // This is prining All the indices. (ignore for now)
  // for (auto i : removedIndices){ 
	// std::cout << i<< " ";
  // }

  // Normal Estimation 
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (bodyFiltered);
  normal_estimator.setKSearch (30);
  normal_estimator.compute (*normals);


  // This was removing some ground points unnecessarily and I could not even print them so ignore
  // pcl::IndicesPtr indices (new std::vector <int>);
  // pcl::PassThrough<pcl::PointXYZ> pass(true);
  // pass.setInputCloud (bodyFiltered);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.0, 0.0001);
  // pass.filter (*indices);

  // // This is prining All the indices. (ignore for now)
  // for (auto i : *indices){ 
	// std::cout << i<< " ";
  // }


  // WE ARE HERE (Also try to understand clusters and how we can use/remove them to get ground planes and other stuff)
  // Play with parameters to get good segmentation results
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (30); // need to check this previously it was 30 or 50 
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (60); // This is also good
  reg.setInputCloud (bodyFiltered);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (6.0 / 180.0 * M_PI); // This is good
  reg.setCurvatureThreshold (0.1);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  // reg.setIndices (clusters[0].indices)

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;


   for (int i=0; i<clusters.size(); i++){ 
     std::cout << "Cluster " << i << " has "<< clusters[i].indices.size () << " points." << std::endl;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane = getGroundCluster(clusters, bodyFiltered);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This has point type XYZRGB lets see if we can do it with Normals as well
// Also see if we can get more than one 


// template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
// pcl::RegionGrowing<PointT, NormalT>::getColoredCloud ()
// {
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

//   if (!clusters_.empty ())
//   {
//     colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

//     srand (static_cast<unsigned int> (time (nullptr)));
//     std::vector<unsigned char> colors;
//     for (std::size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
//     {
//       colors.push_back (static_cast<unsigned char> (rand () % 256));
//       colors.push_back (static_cast<unsigned char> (rand () % 256));
//       colors.push_back (static_cast<unsigned char> (rand () % 256));
//     }

//     colored_cloud->width = input_->width;
//     colored_cloud->height = input_->height;
//     colored_cloud->is_dense = input_->is_dense;
//     for (const auto& i_point: *input_)
//     {
//       pcl::PointXYZRGB point;
//       point.x = *(i_point.data);
//       point.y = *(i_point.data + 1);
//       point.z = *(i_point.data + 2);
//       point.r = 255;
//       point.g = 0;
//       point.b = 0;
//       colored_cloud->points.push_back (point);
//     }

//     int next_color = 0;
//     for (auto i_segment = clusters_.cbegin (); i_segment != clusters_.cend (); i_segment++)
//     {
//       for (auto i_point = i_segment->indices.cbegin (); i_point != i_segment->indices.cend (); i_point++)
//       {
//         int index;
//         index = *i_point;
//         (*colored_cloud)[index].r = colors[3 * next_color];
//         (*colored_cloud)[index].g = colors[3 * next_color + 1];
//         (*colored_cloud)[index].b = colors[3 * next_color + 2];
//       }
//       next_color++;
//     }
//   }

//   return (colored_cloud);
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // pcl::console::print_highlight("Number of segments done is %zu\n",
  //                               static_cast<std::size_t>(clusters.size()));
  // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  // std::cout << "These are the indices of the points of the initial" <<
  //   std::endl << "cloud that belong to the first cluster:" << std::endl;
  // int counter = 0;
  // while (counter < clusters[0].indices.size ())
  // {
  //   std::cout << clusters[0].indices[counter] << ", ";
  //   counter++;
  //   if (counter % 10 == 0)
  //     std::cout << std::endl;
  // }
  // std::cout << std::endl;


  // pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  // pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (colored_cloud));
  // std::vector<int> inliers;
  // pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
  // ransac.setDistanceThreshold (.05);
  // ransac.computeModel();
  // ransac.getInliers(inliers);

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::copyPointCloud (*colored_cloud, inliers, *final);


  // pcl::IndicesPtr indices2 (new std::vector <int>);
  

  
  pcl::visualization::CloudViewer viewer ("Cluster viewer");


  // int32_t rgb = (static_cast<uint32_t>(1) << 16 |
  //     static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(0));
  // for(auto &final: cloud->points) final.rgb=rgb;

  // for(auto &it : final->points){
  //   it.r = 255;
  //   it.b = 0;
  //   it.g = 0;
  // }

  // // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  // std::cout << "These are the indices of the points of the final cloud" <<std::endl;
  // int counter = 0;
  // while (counter < final->points.size ())
  // {
  //   std::cout << final->points[counter] << ", ";
  //   counter++;
  //   if (counter % 10 == 0)
  //     std::cout << std::endl;
  // }
  // std::cout << counter <<std::endl;


  // viewer.showCloud(ground_plane);
  viewer.showCloud(colored_cloud);



  while (!viewer.wasStopped ())
  {
  }

  return (0);
}
