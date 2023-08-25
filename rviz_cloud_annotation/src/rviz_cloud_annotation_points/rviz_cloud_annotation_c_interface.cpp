#include "rviz_cloud_annotation_c_interface.h"

#include "rviz_cloud_annotation_undo.h"
#include "point_neighborhood.h"
#include "rviz_cloud_annotation_definitions.h"

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

static std::shared_ptr<RVizCloudAnnotationUndo> rviz_cloud_annotation_undo;

static std::shared_ptr<RVizCloudAnnotationPoints> rviz_cloud_annotation_points;

static std::shared_ptr<PointNeighborhood> point_neighborhood;

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointXYZRGBNormalCloud;
typedef pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> KdTree;

static std::shared_ptr<KdTree> kdtree;
static PointXYZRGBNormalCloud::Ptr point_cloud;

PointNeighborhood::Conf conf;
static int weight_steps;

extern "C"
{

EXPORT_API int rviz_cloud_annotation_loadcloud(const char * const filename)
{
  point_cloud.reset(new PointXYZRGBNormalCloud);
  if (pcl::io::loadPCDFile(std::string(filename), *point_cloud))
    return int(RvizCloudAnnotationError::FILE_NOT_FOUND);

  kdtree.reset(new KdTree);
  kdtree->setInputCloud(point_cloud);

  try
  {
    conf.searcher = PointNeighborhoodSearch::CreateFromString(PARAM_VALUE_NEIGH_SEARCH_KNN_ATMOST, "10");
  }
  catch (const PointNeighborhoodSearch::ParserException & ex)
  {
    return int(RvizCloudAnnotationError::UNKNOWN_SEARCHER);
  }

  conf.color_importance = PARAM_DEFAULT_COLOR_IMPORTANCE;
  conf.normal_importance = PARAM_DEFAULT_NORMAL_IMPORTANCE;
  conf.position_importance = PARAM_DEFAULT_POSITION_IMPORTANCE;

  conf.max_distance = PARAM_DEFAULT_MAX_DISTANCE;

  weight_steps = PARAM_DEFAULT_WEIGHT_STEPS;

  point_neighborhood.reset(new PointNeighborhood(point_cloud, conf));

  rviz_cloud_annotation_points.reset(new RVizCloudAnnotationPoints(point_cloud->size(), weight_steps, point_neighborhood));

  rviz_cloud_annotation_undo.reset(new RVizCloudAnnotationUndo);
  rviz_cloud_annotation_undo->SetAnnotation(rviz_cloud_annotation_points);

  return int(RvizCloudAnnotationError::NONE);
}

EXPORT_API int rviz_cloud_annotation_load(const char * const filename)
{
  return int(RvizCloudAnnotationError::NONE);
}

EXPORT_API int rviz_cloud_annotation_save(const char * const filename)
{
  return int(RvizCloudAnnotationError::NONE);
}

}
