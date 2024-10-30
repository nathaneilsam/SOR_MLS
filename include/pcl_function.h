#ifndef PCL_FUNCTION_H
#define PCL_FUNCTION_H

#include <thread>
#include <future>
#include <omp.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>///n
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>//n
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/filters/convolution_3d.h>
// #include <pcl/filters/convolution.h>
// #include <pcl/filters/morphological_filter.h>
#include <pcl/features/normal_3d_omp.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr PointCloudINormalPtr; 
typedef pcl::PointCloud<PointType> PointCloudXYZIN;
typedef pcl::PointXYZRGB PointTRGB;
typedef pcl::PointCloud<PointTRGB>::Ptr PointCloudRGBPtr; 

struct PointXYZRGBIL
{
  PCL_ADD_POINT4D                    // quad-word XYZ
  PCL_ADD_RGB
  PCL_ADD_INTENSITY
  int label;                        ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBIL,(float, x, x)(float, y, y)(float, z, z)(uint8_t, b ,b)(uint8_t, g ,g)(uint8_t, r ,r)(float, intensity, intensity)(int, label, label))

// 用于存储体素网格的哈希函数
struct VoxelHash {
    std::size_t operator()(const std::tuple<int, int, int>& key) const {
        return std::get<0>(key) ^ std::get<1>(key) ^ std::get<2>(key);
    }
};



template <typename hashType = float, typename T = void *>
struct Hash3DMap
{
    using hash_3d_T = std::unordered_map<hashType, std::unordered_map<hashType, std::unordered_map<hashType, T>>>;
    hash_3d_T hashmap;

    ~Hash3DMap()
    {
        clear();
    }

    void insert(const hashType &x, const hashType &y, const hashType &z, const T &target)
    {
        hashmap[x][y][z] = target;
    }

    bool exist(const hashType &x, const hashType &y, const hashType &z)
    {
        if(hashmap.find(x) == hashmap.end())
        {
            return false;
        }
        else if(hashmap[x].find(y) ==  hashmap[x].end())
        {
            return false;
        }
        else if(hashmap[x][y].find(z) == hashmap[x][y].end())
        {
            return false;
        }
        return true;
    }

    void clear()
    {
        hashmap.clear();
    }
};

void transformat_cloud(PointCloudINormalPtr incloud_g,pcl::PointCloud<PointXYZRGBIL>::Ptr outcloud );

void splitground_pc(PointCloudINormalPtr incloud,PointCloudINormalPtr gcloud,PointCloudINormalPtr ngcloud);



void MLS_smooth_PointINormalHandle(PointCloudINormalPtr cloud,PointCloudINormalPtr outcloud,int available_opm_num,int Ploynomialorder=2,float radius=0.15
                                    ,bool IsComputeNormal=false);

void voxelFilter(PointCloudINormalPtr incloud,PointCloudINormalPtr outcloud,double leafSize);
void voxelFilter_xs(PointCloudINormalPtr cloud, double filterSize);
//点云法线合并
pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_base_cloudwithnormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals);

void merge_cloudwithnormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals);

//法线计算
pcl::PointCloud<pcl::Normal>::Ptr pcl_feature_normals_k(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in,float K);

// 将点云重新组织到1cm的体素网格中，并且每个体素只保留强度最大的点
PointCloudINormalPtr reorganizePointCloud(PointCloudINormalPtr cloud, float voxel_size = 0.01f);

void calStaticFilter(PointCloudINormalPtr cloud_,int k_,double threshold_,PointCloudINormalPtr output_,int available_opm_num);

//calStaticFilter(cloud, 50, 1.0, cloud_filtered);
void SORFilter_PointINormalHandle(PointCloudINormalPtr incloud,PointCloudINormalPtr outcloud,int MeanK=50,float StdThresh=3.0);//MeanK=10~100 StdThresh=1.0~5.0

void RemoveEgoPoint(PointCloudINormalPtr incloud,PointCloudINormalPtr outcloud,std::vector<double> range);


//PointCloudINormalPtr GaussianFilter_PointINormalHandle(PointCloudINormalPtr cloud,float Segma,float SearchRadius);

pcl::PointCloud<PointType>::Ptr TransformPointCloud(const pcl::PointCloud<PointType>::Ptr inCloud3D, const Eigen::Matrix4d& inMat);



// void Upsample(PointCloudINormalPtr cloud,double sharpness_angle,double edge_sensitivity,
//               double neighbor_radius, int number_of_output_points);
















#endif // PCL_FUNCTION_H