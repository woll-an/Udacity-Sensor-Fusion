// PCL lib Functions for processing point clouds
#include "processPointClouds.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float filterRes,
    Eigen::Vector4f minPoint,
    Eigen::Vector4f maxPoint) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  pcl::VoxelGrid<PointT> sor;
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered(
      new pcl::PointCloud<PointT>);
  sor.setInputCloud(cloud);
  sor.setLeafSize(filterRes, filterRes, filterRes);
  sor.filter(*cloudFiltered);

  typename pcl::PointCloud<PointT>::Ptr cloudRegion(
      new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  std::vector<int> indices;

  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

  for (int point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    std::unordered_set<int> inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  typename pcl::PointCloud<PointT>::Ptr obstacleCloud(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(
      new pcl::PointCloud<PointT>());

  for (int index = 0; index < cloud->points.size(); ++index) {
    if (inliers.count(index) > 0) {
      planeCloud->points.push_back(cloud->points[index]);
    } else {
      obstacleCloud->points.push_back(cloud->points[index]);
    }
  }

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obstacleCloud, planeCloud);
  return segResult;
}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    int maxIterations,
    float distanceTol) {
  std::unordered_set<int> inliersResult;
  int maxInliers = 0;
  srand(time(NULL));

  // For max iterations
  for (int i = 0; i < maxIterations; ++i) {
    int n = cloud->points.size();

    std::unordered_set<int> rdmIndices{};
    while (rdmIndices.size() < 3) {
      rdmIndices.insert(rand() % n);
    }

    auto it = rdmIndices.begin();
    PointT pointA = cloud->points[*it];
    float x1 = pointA.x;
    float y1 = pointA.y;
    float z1 = pointA.z;
    ++it;
    PointT pointB = cloud->points[*it];
    float x2 = pointB.x;
    float y2 = pointB.y;
    float z2 = pointB.z;
    ++it;
    PointT pointC = cloud->points[*it];
    float x3 = pointC.x;
    float y3 = pointC.y;
    float z3 = pointC.z;

    float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float d = -(a * x1 + b * y1 + c * z1);

    std::unordered_set<int> currentInliers;

    for (int j = 0; j < n; ++j) {
      PointT point = cloud->points[j];
      float x = point.x;
      float y = point.y;
      float z = point.z;
      // Measure distance between every point and fitted line
      float distance = std::abs(a * x + b * y + c * z + d) /
                       std::sqrt(a * a + b * b + c * c);
      // If distance is smaller than threshold count it as inlier
      if (distance <= distanceTol) {
        currentInliers.insert(j);
      }
    }
    // Return indicies of inliers from fitted line with most inliers
    if (currentInliers.size() > inliersResult.size()) {
      inliersResult = currentInliers;
    }
  }

  return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  auto inliers = Ransac(cloud, maxIterations, distanceThreshold);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    int pointID,
    std::vector<int>& cluster,
    KdTree<PointT>* tree,
    std::vector<bool>& pointProcessed,
    float distanceTol) {
  pointProcessed[pointID] = true;
  cluster.push_back(pointID);
  std::vector<int> nearby = tree->search(cloud->points[pointID], distanceTol);
  for (int pid : nearby) {
    if (!pointProcessed[pid])
      proximity(cloud, pid, cluster, tree, pointProcessed, distanceTol);
  }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    KdTree<PointT>* tree,
    float distanceTol) {
  int numP = cloud->points.size();

  std::vector<std::vector<int>> clusters;
  std::vector<bool> pointProcessed(numP, false);

  // Iterate through each point
  for (int pid = 0; pid < numP; ++pid) {
    if (!pointProcessed[pid]) {
      std::vector<int> cluster{};
      proximity(cloud, pid, cluster, tree, pointProcessed, distanceTol);
      clusters.push_back(cluster);
    }
  }

  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float clusterTolerance,
    int minSize,
    int maxSize) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  KdTree<PointT>* tree = new KdTree<PointT>();

  for (int i = 0; i < cloud->points.size(); ++i) {
    auto point = cloud->points[i];
    tree->insert(point, i);
  }

  std::vector<std::vector<int>> clustersIndices =
      euclideanCluster(cloud, tree, clusterTolerance);

  for (auto& clusterIndices : clustersIndices) {
    int clusterSize = clusterIndices.size();
    if (clusterSize >= minSize && clusterSize <= maxSize) {
      typename pcl::PointCloud<PointT>::Ptr cloudCluster(
          new pcl::PointCloud<PointT>);
      for (auto& index : clusterIndices) {
        cloudCluster->points.push_back(cloud->points[index]);
      }
      clusters.push_back(cloudCluster);
    }
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
    std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
    std::string dataPath) {
  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}