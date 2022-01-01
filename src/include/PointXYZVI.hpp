#define PCL_NO_PRECOMPILE
//#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

//#include <pcl/point_cloud.h>




struct EIGEN_ALIGN16 PointXYZVI    // enforce SSE padding for correct memory alignment
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float velocity;
  float intensity;
  PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZVI,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, velocity, velocity)
                                   (float, intensity, intensity)
)
/*
int
main (int argc, char** argv)
{
  pcl::PointCloud<PointXYZVI> cloud;
  cloud.resize (2);
  cloud.width = 2;
  cloud.height = 1;

  cloud[0].velocity = 1.f;
  cloud[1].velocity = 2.f;
  cloud[0].x = cloud[0].y = cloud[0].z = 0;
  cloud[1].x = cloud[1].y = cloud[1].z = 3;
  printf("Done");
}
*/