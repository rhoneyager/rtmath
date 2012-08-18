#pragma once

/* This file serves to link the debugging libraries of the Point Cloud Library
 * It is intended for compilations in a pure VS2010 environment - no CMake
 */


#ifndef WITH_CMAKE


#ifdef _DEBUG
// Debug symbols
//#pragma comment(lib, "pcl_apps_debug.lib")
#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_features_debug.lib")
#pragma comment(lib, "pcl_filters_debug.lib")
#pragma comment(lib, "pcl_io_debug.lib")
#pragma comment(lib, "pcl_io_ply_debug.lib")
#pragma comment(lib, "pcl_kdtree_debug.lib")
#pragma comment(lib, "pcl_keypoints_debug.lib")
#pragma comment(lib, "pcl_octree_debug.lib")
#pragma comment(lib, "pcl_registration_debug.lib")
#pragma comment(lib, "pcl_sample_consensus_debug.lib")
#pragma comment(lib, "pcl_search_debug.lib")
#pragma comment(lib, "pcl_segmentation_debug.lib")
#pragma comment(lib, "pcl_surface_debug.lib")
#pragma comment(lib, "pcl_tracking_debug.lib")
#pragma comment(lib, "pcl_visualization_debug.lib")

#pragma comment(lib, "flann_cpp_s-gd.lib")
//#pragma comment(lib, "flann_s-gd.lib")

#pragma comment(lib, "qhull.lib")
#pragma comment(lib, "qhullcpp.lib")
#else
// Release symbols
#pragma comment(lib, "pcl_apps_release.lib")
#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_features_release.lib")
#pragma comment(lib, "pcl_filters_release.lib")
#pragma comment(lib, "pcl_io_release.lib")
#pragma comment(lib, "pcl_io_ply_release.lib")
#pragma comment(lib, "pcl_kdtree_release.lib")
#pragma comment(lib, "pcl_keypoints_release.lib")
#pragma comment(lib, "pcl_octree_release.lib")
#pragma comment(lib, "pcl_registration_release.lib")
#pragma comment(lib, "pcl_sample_consensus_release.lib")
#pragma comment(lib, "pcl_search_release.lib")
#pragma comment(lib, "pcl_segmentation_release.lib")
#pragma comment(lib, "pcl_surface_release.lib")
#pragma comment(lib, "pcl_tracking_release.lib")
#pragma comment(lib, "pcl_visualization_release.lib")

#pragma comment(lib, "flann_cpp_s.lib")
#pragma comment(lib, "flann_s.lib")

#pragma comment(lib, "qhull6.lib")
#pragma comment(lib, "qhullcpp.lib")

#endif

#endif

