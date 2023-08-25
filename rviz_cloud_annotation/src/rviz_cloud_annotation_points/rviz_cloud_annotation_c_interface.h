
#if _MSC_VER // this is defined when compiling with Visual Studio
  #define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
  #define EXPORT_API
#endif

enum class RvizCloudAnnotationError
{
  NONE           = 0,
  FILE_NOT_FOUND = 1,
  UNKNOWN_SEARCHER = 2,
  OTHER_ERROR    = 3,
};

extern "C"
{

EXPORT_API int rviz_cloud_annotation_loadcloud(const char * const filename);

EXPORT_API int rviz_cloud_annotation_load(const char * const filename);

EXPORT_API int rviz_cloud_annotation_save(const char * const filename);

}
