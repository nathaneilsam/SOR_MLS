#include"pcl_function.h"

void transformat_cloud(PointCloudINormalPtr incloud_g,pcl::PointCloud<PointXYZRGBIL>::Ptr outcloud )
{
    int pc_num=incloud_g->size();
    // for(int i=0;i<pc_num1;++i)
    // {
    //    PointXYZRGBIL point_rgbil;
    //    point_rgbil.x= incloud_ng->points[i].x;
    //    point_rgbil.y= incloud_ng->points[i].y;
    //    point_rgbil.z= incloud_ng->points[i].z;
    //    point_rgbil.intensity= incloud_ng->points[i].intensity;
    //    point_rgbil.label= int(incloud_ng->points[i].curvature);
    //    outcloud->push_back(point_rgbil);
    // }
    for(int i=0;i<pc_num;++i)
    {
       PointXYZRGBIL point_rgbil;
       point_rgbil.x= incloud_g->points[i].x;
       point_rgbil.y= incloud_g->points[i].y;
       point_rgbil.z= incloud_g->points[i].z;
       point_rgbil.intensity= incloud_g->points[i].intensity;
       point_rgbil.label= ceil(incloud_g->points[i].curvature);
       outcloud->push_back(point_rgbil);
    }
}
void splitground_pc(PointCloudINormalPtr incloud,PointCloudINormalPtr gcloud,PointCloudINormalPtr ngcloud)
{
    int pc_num=incloud->size();
    for(int i=0;i<pc_num;++i)
    {
       PointType curpoint;
       if(incloud->points[i].curvature==1)
            gcloud->push_back(incloud->points[i]);
        else
            ngcloud->push_back(incloud->points[i]);
    }
}



void MLS_smooth_PointINormalHandle(PointCloudINormalPtr cloud,PointCloudINormalPtr outcloud,
                                   int available_opm_num,int Ploynomialorder,float radius,bool IsComputeNormal)
{
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>);
    pcl::MovingLeastSquares<pcl::PointXYZINormal,pcl::PointXYZINormal> mls;
    // mls.setComputeNormals(IsComputeNormal); //false);// //设置在最小二乘计算中需要进行法线估计
	mls.setInputCloud(cloud);
    mls.setPolynomialOrder(Ploynomialorder);
	mls.setPolynomialFit(true); //设置false，可以加速平滑
	mls.setSearchMethod(tree);
    int num_thread=std::thread::hardware_concurrency();
    mls.setNumberOfThreads(available_opm_num);
	mls.setSearchRadius(radius); //单位m，设置用于拟合的K近邻半径
    // mls.setUpsamplingMethod(mls.UpsamplingMethod::RANDOM_UNIFORM_DENSITY);
    // mls.setPointDensity(100);
    // mls.setUpsamplingMethod(mls.UpsamplingMethod::SAMPLE_LOCAL_PLANE);
    // // // //mls.setDistinctCloud(cloud);
    // mls.setUpsamplingRadius(radius);
    // mls.setUpsamplingStepSize(0.9*radius);
    mls.process(*outcloud); 
    return;
}

void voxelFilter(PointCloudINormalPtr incloud,PointCloudINormalPtr outcloud,double leafSize)
{
    // 使用普通的VoxelGrid滤波算法求得每个体素网格的质 心坐标
    pcl::VoxelGrid<PointType> voxelGridFilter;
    voxelGridFilter.setInputCloud(incloud);
    voxelGridFilter.setLeafSize(leafSize, leafSize, leafSize);
    voxelGridFilter.filter(*outcloud);
}
//点云法线合并
pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_base_cloudwithnormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::concatenateFields(*cloud,*normals,*cloud_normals);
    return cloud_normals;
}

void merge_cloudwithnormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  if (cloud->size () != normals->size ())
  {
    PCL_ERROR (" The number of points in the two input datasets differs!\n");
    return;
  }
  for(int i = 0;i<cloud->size();++i)
  {
    cloud->points[i].normal_x = normals->points[i].normal_x;
    cloud->points[i].normal_y = normals->points[i].normal_y;
    cloud->points[i].normal_z = normals->points[i].normal_z;
  }
}
//法线计算
pcl::PointCloud<pcl::Normal>::Ptr pcl_feature_normals_k(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in,float K)
{
    pcl::NormalEstimationOMP<pcl::PointXYZINormal, pcl::Normal> ne;
    ne.setInputCloud(cloud_in);
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(K);//设置K近邻的个数
    ne.compute(*normals);
    return normals;
}

// 将点云重新组织到1cm的体素网格中，并且每个体素只保留强度最大的点
PointCloudINormalPtr reorganizePointCloud(PointCloudINormalPtr cloud, float voxel_size) {
    // 使用哈希表存储每个体素中的最大强度点
    std::unordered_map<std::tuple<int, int, int>, pcl::PointXYZINormal, VoxelHash> voxel_map;

    // 并行化处理点云，计算每个点所属的体素
    #pragma omp parallel for
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        auto& point = cloud->points[i];
        int voxel_x = std::floor(point.x / voxel_size);
        int voxel_y = std::floor(point.y / voxel_size);
        int voxel_z = std::floor(point.z / voxel_size);

        auto voxel_key = std::make_tuple(voxel_x, voxel_y, voxel_z);

        #pragma omp critical  // 防止多个线程同时修改哈希表
        {
            // 如果该体素中还没有点，或者该点的强度更大，则更新
            if (voxel_map.find(voxel_key) == voxel_map.end() || point.intensity > voxel_map[voxel_key].intensity) {
                voxel_map[voxel_key] = point;
            }
        }
    }

    // 创建新的点云，包含每个体素中的最大强度点
    PointCloudINormalPtr new_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    for (const auto& voxel : voxel_map) {
        new_cloud->points.push_back(voxel.second);
    }

    new_cloud->width = new_cloud->points.size();
    new_cloud->height = 1;  // 无序点云
    new_cloud->is_dense = true;
    return new_cloud;
}

void calStaticFilter(PointCloudINormalPtr cloud_,int k_,double threshold_,PointCloudINormalPtr output_,int available_opm_num)
{
    pcl::search::KdTree<pcl::PointXYZINormal> tree;
    tree.setInputCloud(cloud_);
    int nCloud = cloud_->points.size();
          
    std::vector<float>aveDistance;
    aveDistance.resize(nCloud,0);
    double scale = 1.0 / (double)k_;

    int thread_count = std::thread::hardware_concurrency();
#if defined(_WIN32) || defined(_WIN64)
    SYSTEM_INFO si;
    ::GetSystemInfo(&si);
    thread_count = si.dwNumberOfProcessors;
#endif
    if (thread_count > 8)
    {
        thread_count = thread_count * 0.8;
    }
    //std::cout<<"start openmp 2,num: "<<thread_count<<std::endl;
    // omp_set_num_threads(thread_count);
    omp_set_num_threads(available_opm_num);
#pragma omp parallel for schedule(static)    
    for (int i = 0; i < nCloud; i++)
    {
        pcl::PointXYZINormal p = cloud_->points[i];
        std::vector<float>distance;
        pcl::Indices indi;pcl::IndicesPtr indis;
        int n = tree.nearestKSearch(p, k_, indi, distance);
        if (n > 0)
        {
            double sum = 0;
            for (int j = 0; j < k_; j++)
            {
                sum += distance[j];                
            }
            aveDistance[i] = sum * scale;
        }
    }   
    double sumAve = 0;
    for (int i = 0; i < nCloud; i++)
    {
         sumAve += aveDistance[i];
    }
    double mean = sumAve / (double)nCloud;
    double variance = 0.0;
    for (int i = 0; i < nCloud; i++)
    {
        variance += pow(aveDistance[i] - mean, 2);
    }
    double standard_deviation = sqrt(variance/nCloud);
    double segThreshold = 0;
    if (standard_deviation > mean)
    {
        segThreshold = mean + mean*threshold_;
    }
    else
    {
        segThreshold = mean + standard_deviation * threshold_;
    }
    std::vector<int>vecIndex;
    vecIndex.resize(nCloud, 0);
    int nFilter = 0;
    for (int i = 0; i < nCloud; i++)
    {
        if (aveDistance[i] < segThreshold)
        {
            vecIndex[i] = 1;
            nFilter++;
        }
    }
    output_->points.resize(nFilter);
    for (int i = 0, j = 0; i < nCloud; i++)
    {
        if (vecIndex[i] > 0 && j < nFilter)
        {
            output_->points[j] = cloud_->points[i];
            j++;
        }
    }
    output_->width = nFilter;
    output_->height = 1;
}
//calStaticFilter(cloud, 50, 1.0, cloud_filtered);
void SORFilter_PointINormalHandle(PointCloudINormalPtr incloud,PointCloudINormalPtr outcloud,int MeanK,float StdThresh)//MeanK=10~100 StdThresh=1.0~5.0
{
    
    // PointCloudINormalPtr cloud_filtered(new pcl::PointCloud<pcl::PointXYZINormal
    
     // 创建统计学离群点移除过滤器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
    sor.setInputCloud(incloud);
    // 设置过滤邻域K三
    sor.setMeanK(MeanK);
    // 设置标准差的系数, 值越大，丢掉的点越少
    sor.setStddevMulThresh(StdThresh);
    sor.filter(*outcloud);
    return;
    // 保存去掉离群点之后的点云
    // return cloud_filtered;
    // 取反 保存被去掉离群点
    // sor.setNegative(true);
    // sor.filter(*cloud_filtered);
}
void RemoveEgoPoint(PointCloudINormalPtr incloud,PointCloudINormalPtr outcloud,std::vector<double> range)
{
    // outcloud->clear();
    for(int i=0;i<incloud->size();++i)
    {
        if(incloud->points[i].x>range[0]&&incloud->points[i].x<range[1]&&incloud->points[i].y>range[2]
        &&incloud->points[i].y<range[3]&& incloud->points[i].z>range[4]&&incloud->points[i].z<range[5]
        &&incloud->points[i].curvature==0||(incloud->points[i].x*incloud->points[i].x+incloud->points[i].y*incloud->points[i].y)>45*45)
        {
            continue;
        }
        else outcloud->push_back(incloud->points[i]);
    }
}

// PointCloudINormalPtr GaussianFilter_PointINormalHandle(PointCloudINormalPtr cloud,float Segma,float SearchRadius){
//     pcl::PointCloud<pcl::PointXYZINormal>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
//      // Apply Gaussian smoothing filter
//     pcl::filters::GaussianKernel<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr gaussian_kernel (new pcl::filters::GaussianKernel<pcl::PointXYZINormal, pcl::PointXYZINormal>);
//     // (*gaussian_kernel).setInputCloud(cloud);
//     (*gaussian_kernel).setSigma(4);//0.4
//     (*gaussian_kernel).setThresholdRelativeToSigma(4);
//     (*gaussian_kernel).setThreshold(0.05);//设置距离阈值，若点间距离大于阈值则不予考虑

//     pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZINormal>);
//     (*kdtree).setInputCloud(cloud);
//     std::cout << "KdTree made" << std::endl;
//     pcl::filters::Convolution3D<pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::filters::GaussianKernel<pcl::PointXYZINormal,pcl::PointXYZINormal>> convolution;
    

//     convolution.setInputCloud(cloud);
//     convolution.setKernel(*gaussian_kernel);//设置卷积核
//     convolution.setSearchMethod(kdtree);
// 	convolution.setNumberOfThreads(2);
// 	convolution.setRadiusSearch(SearchRadius);
// 	std::cout << "Convolution Start" << std::endl;
//     convolution.convolve(*smoothed_cloud);
//     std::cout << "filter made" << std::endl;
//     return smoothed_cloud;
// }


pcl::PointCloud<PointType>::Ptr TransformPointCloud(const pcl::PointCloud<PointType>::Ptr inCloud3D, const Eigen::Matrix4d& inMat)
{
        size_t cloudSize = inCloud3D->size();
    pcl::PointCloud<PointType>::Ptr outCloud(new pcl::PointCloud<PointType>);
    outCloud->resize(cloudSize);

    for (size_t i = 0; i < cloudSize; ++i)
    {
        PointType point, pointOut;
        point.x = inCloud3D->points[i].x;
        point.y = inCloud3D->points[i].y;
        point.z = inCloud3D->points[i].z;//add backpack heigh?
        point.intensity = inCloud3D->points[i].intensity;
        point.curvature= inCloud3D->points[i].curvature;

        pointOut.x =
            float(inMat(0, 0) * double(point.x) + inMat(0, 1) * double(point.y) + inMat(0, 2) * double(point.z) + inMat(0, 3));  // m
        pointOut.y =
            float(inMat(1, 0) * double(point.x) + inMat(1, 1) * double(point.y) + inMat(1, 2) * double(point.z) + inMat(1, 3));  // m
        pointOut.z =
            float(inMat(2, 0) * double(point.x) + inMat(2, 1) * double(point.y) + inMat(2, 2) * double(point.z) + inMat(2, 3));  // m

        pointOut.intensity = point.intensity;
        pointOut.curvature=point.curvature;
        outCloud->points[i] = pointOut;
    }
    return outCloud;

}

template <typename T = int64_t>
struct VoxelId
{
public:
  T x;
  T y;
  T z;

  VoxelId(T vx=0, T vy=0, T vz=0): x(vx), y(vy), z(vz){}

  bool operator == (const VoxelId &other) const
  {
    return (x==other.x && y==other.y && z==other.z);
  }

  bool operator<(const VoxelId& p) const
  {
    if (x < p.x) return true;
    if (x > p.x) return false;
    if (y < p.y) return true;
    if (y > p.y) return false;
    if (z < p.z) return true;
    if (z > p.z) return false;
  }
};

template <typename T1, typename T2>
VoxelId<T2> calculateVoxelId(T1 x, T1 y, T1 z, T1 unitSize)
{
    double vidx = x / unitSize;
    double vidy = y / unitSize;
    double vidz = z / unitSize;
    if (vidx < 0) vidx -= 1;
    if (vidy < 0) vidy -= 1;
    if (vidz < 0) vidz -= 1;
    return VoxelId<T2>(vidx, vidy, vidz);
}

void voxelFilter_xs(PointCloudINormalPtr cloud,double filterSize)
{
    if (cloud == nullptr || cloud->points.empty())
        return;
    PointCloudINormalPtr outputCloud(new PointCloudXYZIN);
    outputCloud->points.resize(cloud->points.size());
    int outputCount = 0;
    VoxelId<int64_t> voxelId;
    Hash3DMap<int64_t, float> occupyMap;
    for (int i=0; i<cloud->points.size(); ++i)
    {
        voxelId = calculateVoxelId<double, int64_t>(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, filterSize);
        if (!occupyMap.exist(voxelId.x, voxelId.y, voxelId.z))
        {
            outputCloud->points[outputCount++] = cloud->points[i];
            occupyMap.insert(voxelId.x, voxelId.y, voxelId.z, cloud->points[i].intensity);
        }
    }
    outputCloud->points.resize(outputCount);
    cloud->points.swap(outputCloud->points);
    cloud->height = 1;
    cloud->width = cloud->points.size();
}





// void Upsample(PointCloudINormalPtr cloud,double sharpness_angle,double edge_sensitivity,
//               double neighbor_radius, int number_of_output_points)
// {
//     if(cloud->size()==0)
//     {
//         std::cout<<"inpout cloud is empty,cannot upsample"<<std::endl;
//     }
//     else{
//        //计算法线
//        int nancount=0;
//         pcl::PointCloud<pcl::Normal>::Ptr normals = pcl_feature_normals_k(cloud->makeShared(),10);
//         for (const auto& normal : *normals) {
//             if (std::isnan(normal.normal_x) || std::isnan(normal.normal_y) || std::isnan(normal.normal_z))
//             {
//                 // std::cout<<"normal has Nan "<<std::endl;
//                 nancount++;
//             }
//         }
//         std::cout<<"pointcloud num:"<<cloud->size()<<" total nan num:"<<nancount<<std::endl;
//         //PointCloudINormalPtr cloud_with_normals=pcl_base_cloudwithnormals(cloud->makeShared(),normals);
//         merge_cloudwithnormals(cloud,normals);
//         PointCloudINormalPtr cloud_with_normals=cloud;
//         //转换为cgal格式
//         std::vector<PointVectorPair> cgal_cloud;
//         for(int i=0;i<cloud_with_normals->size();i++)
//         {

//            double px=cloud_with_normals->points[i].x;
//            double py=cloud_with_normals->points[i].y;
//            double pz=cloud_with_normals->points[i].z;
//            double nx=cloud_with_normals->points[i].normal_x;
//            double ny=cloud_with_normals->points[i].normal_y;
//            double nz=cloud_with_normals->points[i].normal_z;
//            if (std::isnan(nx) || std::isnan(ny) || std::isnan(nz))
//             {
//             //    std::cout<<"normal has Nan "<<std::endl;
//                continue;
//             } 
//         //    double intensity = cloud_with_normals->points[i].intensity;
//         //    double label = cloud_with_normals->points[i].curvature;
//         //    PointWithAttributes point_attr(CGAL_Point(px, py, pz), intensity, label);
//         //    cgal_cloud.push_back(PointVectorPair(point_attr,Vector(nx,ny,nz)));
//             cgal_cloud.push_back(PointVectorPair(CGAL_Point(px, py, pz),Vector(nx,ny,nz)));
//         }

//         cgal_cloud=cgal_unsampling(cgal_cloud,sharpness_angle, edge_sensitivity,neighbor_radius,  number_of_output_points*cgal_cloud.size());

//         cloud->clear();

//         for(int i=0;i<cgal_cloud.size();i++)
//         {
//             pcl::PointXYZINormal p;
//             // PointWithAttributes point_attr_temp = cgal_cloud[i].first;
//             CGAL_Point p_temp= cgal_cloud[i].first; //point_attr_temp.point;//
//             p.x=p_temp.hx();
//             p.y=p_temp.hy();
//             p.z=p_temp.hz();
//             // p.intensity = point_attr_temp.intensity;
//             // p.curvature = point_attr_temp.label;
//             cloud->push_back(p);
//         }
//     }

// }




