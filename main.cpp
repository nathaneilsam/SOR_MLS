#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <ctime>
#include <cstdlib>
#include <thread>
#include <future>
#include <omp.h>
#include <deque>
// #include <experimental/filesystem>
#include <fstream>
#include<chrono>
#include <condition_variable>


#include <cassert>
// #include <boost/thread/thread.hpp>

#include <nlohmann/json.hpp>
#include "AxisTran.h"
#include "pcl_function.h"
#include "xsproto_common_common.h"
#include "buildSpatialMap.h"

using Json = nlohmann::json;

#define CC_APP_XXX_VERSION_NAME  "modular version:"
#define CC_APP_XXX_MAJOR_VERSION "2"
#define CC_APP_XXX_MINOR_VERSION "0"
#define CC_APP_XXX_PATCH_VERSION "0"

int available_opm_num=1;
std::string outname = "",jointname = "Cloud_joint",inname = "";
bool save_joint_flage =false;
Eigen::Matrix4d lidar_params = Eigen::Matrix4d::Zero();
bool is_lidar_params_loaded = false;  // 标志变量
std::mutex mtx_buffer, load_mutex,mtx_process1,mtx_process2;
std::deque<boost::shared_ptr<PointCloudXYZIN>> load_buffer_pcs;
// std::deque<boost::shared_ptr<PointCloudXYZIN>> buffer_ng_pcs;
std::deque<boost::shared_ptr<pcl::PointCloud<PointXYZRGBIL>>> save_buffer_pcs;
std::deque<std::string> buffer_patch_names;
std::deque<std::string> buffer_save_names;
// deque<boost::shared_ptr<PointCloudXYZI>>save_buffer_pcs;
bool buffer_load_finish=false,buffer_deal_finish=false;
std::condition_variable buffer_cv;
// 定义缓冲区大小限制
size_t BUFFER_SIZE_LIMIT = 5;

//useful params
int grid_size= 50;
int image_link_radius =50;
bool use2000=false,use_self_point_filter =true;
double gauss_x0=0.0,gauss_y0=0.0;
std::vector<double> EgoRange;

int K = 50;
float K_threshold = 1.0;
int Ploynomialorder=2;
float search_radius=0.1;
double leaf_size = 0.05;
int save_patch =0,deal_patch=0;
int total_patch =0;



PointCloudINormalPtr current_frame_cloud(new pcl::PointCloud<pcl::PointXYZINormal>()),
                    current_frame_cloud_new(new pcl::PointCloud<pcl::PointXYZINormal>());

void initLog(std::string log_path)
{
    if(log_path.empty()){
        std::cerr << "Input error ,please provider log path" << std::endl;
        exit(0);
    }
    SINGLETON_LOG.setTraceLog("SOR_MLSl", log_path.c_str(), 0, 2, 3, 0);
    LogInfo("SOR_MLS Model Start\n");
    LogInfo("XS_LOG_PATH = %s\n",log_path.c_str());
}

std::unordered_map<std::string, std::string> read_params(std::string file_path)
{
    std::ifstream file(file_path);
    std::unordered_map<std::string, std::string> configMap;

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return configMap;
    }
    std::string line;
    while (getline(file, line)) {
        // 忽略以#开头的行
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        std::string key, value;
        if (std::getline(iss, key, '=') && std::getline(iss, value)) {
            // 去除字符串前后的空白字符
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            configMap[key] = value;
        }
    }
    file.close();
    return configMap;
}


bool LoadCloudTxt(std::string CloudPath, std::map<uint64_t, std::string>&cloudPcdPaths)
{
     std::ifstream file(CloudPath);  // 替换为实际的文件路径
    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
        file.close();
        return false;
    }
    std::string line;
    std::string idx,cloudfile_path ;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string idx,cloudfile_path ;
        iss>>idx>>cloudfile_path;
        // std::cout<<"idx:"<<idx<<" pc_filename:"<<cloudfile_path<<std::endl;
        cloudPcdPaths.insert({uint64_t(std::stoull(idx)),cloudfile_path});
    }
    file.close();
    return true;
}
Eigen::Quaterniond RPYDatatoQuatData(double roll,double pitch,double yaw)
{
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle;

    quaternion.normalize();

    return quaternion;
}
bool LoadXstreeFrameTrackJsonPose(std::string trackPath,std::vector<std::pair<std::string,std::vector<double>>> &poseVecVec)
{
    printf("Load Track Path: %s \n", trackPath.c_str());
    // 加载Cloud.txt 文件
    std::map<uint64_t, std::string> cloudPcdPaths;
    if (!LoadCloudTxt(trackPath + "/Cloud.txt", cloudPcdPaths))
    {
        printf("[LoadCloudTxt] Failed to Load %s \n", (trackPath + "/Cloud.txt").c_str());
        return false;
    }
    else
    {
        printf("[LoadCloudTxt] Load %s Successed.\n", (trackPath + "/Cloud.txt").c_str());

    }
    std::ifstream fileStream(trackPath + "/FrameTrack.json");
    if (!fileStream)
    {
        std::cerr << "FrameTrack.json file error" << std::endl;
        return false;
    }
    else
    {
        Json jsonContent;
        fileStream >> jsonContent;
        if (jsonContent.is_discarded())
        {
            std::cerr << "json data is error" << std::endl;
            return false;
        }
        if (jsonContent.end() == jsonContent.find("Name"))
        {
            return false;
        }

        printf("----> start Load Json!\n");
        Json vertexList = jsonContent["VertexList"]["FramePoseVertex"];
        // 初始化一个时间戳编号的Mapper
        // std::map<std::string, std::vector<double>> timeIdxFrameList;  // 编号为时间戳编号
        // ========== 导入所有的FrameVertex（除KeyIdx以外） ==========
        if (vertexList.is_array())
        {
            printf("VertexList size: %zu \n", vertexList.size());
            for (size_t i = 0; i < vertexList.size(); ++i)
            {
                double pose_x,pose_y,pose_z,roll,pitch,yaw;
                Json poseJson = vertexList.at(i);
                uint64_t        curTimeIdx = poseJson["Idx"].get<uint64_t>();

                if (cloudPcdPaths.find(curTimeIdx) == cloudPcdPaths.end())
                {
                    printf("Cannot Find Cloud Pcd Path of Frame %lu \n", curTimeIdx);
                    return false;
                }
                std::string cur_cloudfile = cloudPcdPaths[curTimeIdx];
                pose_x   = poseJson["Translation"][0].get<double>();
                pose_y   = poseJson["Translation"][1].get<double>();
                pose_z   = poseJson["Translation"][2].get<double>();

                roll  = poseJson["Rotation"][0].get<double>();
                pitch = poseJson["Rotation"][1].get<double>();
                yaw   = poseJson["Rotation"][2].get<double>();
                Eigen::Quaternion quat = RPYDatatoQuatData(roll,pitch,yaw);
                std::vector<double> poseVec={ pose_x,pose_y,pose_z,quat.w(),quat.x(),quat.y(),quat.z()};
                poseVecVec.push_back(std::make_pair(cur_cloudfile, poseVec));
            }
            printf("read successfully frametarck.json ");
        }
    }
    std::ifstream fileStream2(trackPath + "/KeyTrack.json");
    if (!fileStream2)
    {
        std::cerr << "file error" << std::endl;
        return false;
    }
    else
    {
        Json jsonContent;
        fileStream2 >> jsonContent;
        // 读取KeyFrame节点的值
        Json keyVertexList = jsonContent["VertexList"]["KeyPoseVertex"];
        if (keyVertexList.is_array())
        {
             printf("keyVertexList size: %zu \n", keyVertexList.size());
            for (size_t i = 0; i < keyVertexList.size(); ++i)
            {
                double pose_x,pose_y,pose_z,roll,pitch,yaw;
                Json poseJson = keyVertexList.at(i);
                uint64_t curTimeIdx = poseJson["Idx"].get<uint64_t>();
                if (cloudPcdPaths.find(curTimeIdx) == cloudPcdPaths.end())
                {
                    printf("Cannot Find Cloud Pcd Path of Frame %lu \n", curTimeIdx);
                    return false;
                }
                std::string cur_cloudfile = cloudPcdPaths[curTimeIdx];
                pose_x   = poseJson["Translation"][0].get<double>();
                pose_y   = poseJson["Translation"][1].get<double>();
                pose_z   = poseJson["Translation"][2].get<double>();

                roll  = poseJson["Rotation"][0].get<double>();
                pitch = poseJson["Rotation"][1].get<double>();
                yaw   = poseJson["Rotation"][2].get<double>();
                Eigen::Quaternion quat = RPYDatatoQuatData(roll,pitch,yaw);
                std::vector<double> poseVec={ pose_x,pose_y,pose_z,quat.w(),quat.x(),quat.y(),quat.z()};
                poseVecVec.push_back(std::make_pair(cur_cloudfile, poseVec));
            }
        }
        printf("read successfully keytrack.json");
    }   
    return true;
}

bool LoadPgodataInfo(std::string filepath,std::vector<VertexData>&vertexDataList,std::vector<double>&range)
{
    double min_x=DBL_MAX,max_x=DBL_MIN,min_y=DBL_MAX,max_y=DBL_MIN,min_z=DBL_MAX,max_z=DBL_MIN;
    std::ifstream file(filepath);
    std::string line;
    // std::vector<VertexData> vertexDataList;
    if (file.is_open()) {
        while (getline(file, line)) 
        {
            // 忽略以#开头的行
            if (line.empty() || line[0] == '#') {
                continue;
            }
            std::istringstream iss(line);
            std::string token,cloud_path;
            VertexData vertexData;
            // 按顺序读取每个字段
            std::getline(iss, token, ' '); // VERTEX
            if(token !="VERTEX")
                continue;
            std::getline(iss, token, ' '); // FrameIdx
            vertexData.id = std::stoi(token);
            std::getline(iss, vertexData.Timestamp, ' '); // TimeStamp
            std::getline(iss, token, ' '); // KeyFlag
            std::getline(iss, vertexData.Bagname, ' '); // BagName
            iss >> vertexData.Tx >> vertexData.Ty >> vertexData.Tz
                >> vertexData.Rx >> vertexData.Ry >> vertexData.Rz;
            std::getline(iss, token, ' '); // 跳过空格
            std::getline(iss, cloud_path,' '); // CloudPath
            size_t pos = cloud_path.find("Cloud");
            if(pos!=std::string::npos)
            {
                cloud_path.replace(pos,strlen("Cloud"),inname);
            }
            vertexData.CloudPath = cloud_path;
            // 将读取的数据添加到列表中
            vertexDataList.push_back(vertexData);
            min_x=min_x>vertexData.Tx?vertexData.Tx:min_x;
            max_x=max_x<vertexData.Tx?vertexData.Tx:max_x;
            min_y=min_y>vertexData.Ty?vertexData.Ty:min_y;
            max_y=max_y<vertexData.Ty?vertexData.Ty:max_y;
            min_z=min_z>vertexData.Tz?vertexData.Tz:min_z;
            max_z=max_z<vertexData.Tz?vertexData.Tz:max_z;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filepath << std::endl;
        return false;
    }
    range.assign({min_x,max_x,min_y,max_y,min_z,max_z});
    return true;
}

void ConvertGaosito2000(double dbX,double dbY,double dbZ,double&dbCX,double&dbCY)
{
    Eigen::Vector3d in(dbX,dbY,dbZ);
    Eigen::Vector3d out;
    AxisTran::pos_Guass_to_WGS84(in,&out ,-1);
    double dbLatitude,dbLongitude;
    dbLatitude = out[0]*180/M_PI;
    dbLongitude = out[1]*180/M_PI;
    AxisTran::GuassLLH trans;
    trans.TransformWgs84To2000(dbLongitude,dbLatitude,dbCX,dbCY);
}
void ConvertWGS84to2000(double dbLongitude,double dbLatitude,double&dbCX,double&dbCY,int zone)
{
    AxisTran::GuassLLH trans;
    trans.TransformWgs84To2000(dbLongitude,dbLatitude,dbCX,dbCY);
}
void ConvertPgoDataAsCgcs2000(Eigen::Matrix4d& poseMat, double dbOffsetX,double dbOffsetY,double dbOffsetCX,double dbOffsetCY)
{
    double dbX,dbY,dbZ;
    double dbCX,dbCY;
    dbX = dbOffsetX +poseMat(0,3);
    dbY = dbOffsetY +poseMat(1,3);
    dbZ =  poseMat(2,3);
    ConvertGaosito2000(dbX,dbY,dbZ,dbCX,dbCY);
    dbX = dbCX - dbOffsetCX;
    dbY = dbCY - dbOffsetCY;
    poseMat(0,3) = dbX;
    poseMat(1,3) = dbY;
}


Eigen::Matrix4d readPosefile(std::string posefilename,Eigen::Matrix4d &poseMat)
{
    FILE* fpRead = fopen(posefilename.c_str(), "r");
            if (fpRead == NULL)
            {
                    fclose(fpRead);
            }
                //加载点云pose
            double currTime;double pose_x,pose_y,pose_z,quat_x,quat_y,quat_z,quat_w; int currtindex;
            while (!feof(fpRead))
            {
                fscanf(fpRead, "scantime: %lf\n", &currTime);
                if(posefilename.substr(posefilename.size() - 5) == ".info")
                fscanf(fpRead, "scanidx: %lf\n",&currtindex);
                fscanf(fpRead, "odom pose: %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", &pose_x, &pose_y, &pose_z,
                    &quat_x, &quat_y, &quat_z, &quat_w);
                break;
            }
            fclose(fpRead);
            // Eigen::Matrix4d poseMat = Eigen::Matrix4d::Identity();
            Eigen::Matrix3d rotmat = Eigen::Quaterniond(quat_w, quat_x, quat_y,quat_z).normalized().matrix();
            poseMat.block<3, 3>(0, 0) = rotmat;
            poseMat(0, 3) = pose_x;
            poseMat(1, 3) = pose_y;
            poseMat(2, 3) =pose_z;  
            // return poseMat;
}

void processRange(std::vector<std::vector<std::string>>trace_idexs,std::map<std::string ,std::string> filenames,
                std::map<std::string ,std::string> posefilenames,std::vector<XYZID<>> locations,std::string file_path,int grid_size,bool t2000,\
                double dbOffsetX,double dbOffsetY,double dbOffsetZ=0.0)//,std::string params_path)
    {
        // 使用 std::lock_guard 自动释放锁
        // std::lock_guard<std::mutex> lock(mtx);
        int64_t idx,idy,idz,ids,ik=0;
        // clock_t start_t=clock();
        auto start_t = std::chrono::high_resolution_clock::now();
        double dbOffsetCX,dbOffsetCY;
        if (t2000)
        {
            ConvertGaosito2000(dbOffsetX, dbOffsetY,dbOffsetZ,dbOffsetCX,dbOffsetCY);
        }
        for(int i=0;i<trace_idexs.size();++i)
        {
            clock_t start_p=clock();
            std::vector<std::string> trace_index;
            trace_index=trace_idexs[i];
            if(trace_index.size()==0)continue;
            PointCloudINormalPtr patch_cloud_g(new pcl::PointCloud<pcl::PointXYZINormal>),patch_cloud_ng(new pcl::PointCloud<pcl::PointXYZINormal>);
            for(int k=0;k<trace_index.size();++k)
            {
                const std::string filename=filenames[trace_index[k]];
                const std::string posefilename=posefilenames[trace_index[k]];
                PointCloudINormalPtr current_frame_cloud(new pcl::PointCloud<pcl::PointXYZINormal>),
                                     current_frame_cloud_g_new(new pcl::PointCloud<pcl::PointXYZINormal>),
                                     current_frame_cloud_ng_new(new pcl::PointCloud<pcl::PointXYZINormal>);
                PointCloudINormalPtr cloud_g(new pcl::PointCloud<pcl::PointXYZINormal>),cloud_ng(new pcl::PointCloud<pcl::PointXYZINormal>);
                pcl::io::loadPCDFile<pcl::PointXYZINormal>(filename, *current_frame_cloud);
 

                splitground_pc(current_frame_cloud,cloud_g,cloud_ng);
                RemoveEgoPoint(cloud_ng,current_frame_cloud_ng_new,EgoRange);
                RemoveEgoPoint(cloud_g,current_frame_cloud_g_new,EgoRange);
                if(k%300==0)pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZINormal>(file_path+"/single"+trace_index[k]+".pcd", *current_frame_cloud_ng_new+*current_frame_cloud_g_new);
                Eigen::Matrix4d poseMat =Eigen::Matrix4d::Identity();
                readPosefile(posefilename,poseMat);
                if(t2000)
                {
                    ConvertPgoDataAsCgcs2000(poseMat,dbOffsetX,dbOffsetY,dbOffsetCX,dbOffsetCY);
                }

                //
                // poseMat*=lidar_params;

                *patch_cloud_ng+=*TransformPointCloud(current_frame_cloud_ng_new,poseMat);
                *patch_cloud_g+=*TransformPointCloud(current_frame_cloud_g_new,poseMat);
                // *patch_cloud+=*current_frame_cloud;
                current_frame_cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());cloud_g.reset();cloud_ng.reset();
                current_frame_cloud_ng_new.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
                current_frame_cloud_g_new.reset();
            }

            PointCloudINormalPtr patch_cloud_f(new pcl::PointCloud<pcl::PointXYZINormal>);
            idx =locations[ik].xid;idy=locations[ik].yid;idz=locations[ik].zid;ids=locations[ik].sid;
            std::string patch_name=std::to_string(idx)+"-"+std::to_string(idy)+"-"+std::to_string(idz)+"-"+std::to_string(ids);
            ik++;
            std::cout<<"sorfilter start, pc num:"<<patch_cloud_ng->size()<<std::endl;
            // clock_t start=clock();
            // 开始计时
            auto start = std::chrono::high_resolution_clock::now();
            PointCloudINormalPtr patch_cloud_filtered(new pcl::PointCloud<pcl::PointXYZINormal>);
            // patch_cloud_filtered=SORFilter_PointINormalHandle(patch_cloud_f,50,3);
            // SORFilter_PointINormalHandle(patch_cloud_g,patch_cloud_filtered,10,1);
            calStaticFilter(patch_cloud_ng, 50, 1.0, patch_cloud_filtered,available_opm_num);
            // patch_cloud_filtered= GaussianFilter_PointINormalHandle(patch_cloud_f,4,0.1);
            patch_cloud_f.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
            // boost::filesystem::path p0(file_path+"/jointCloud");  
            // if (!boost::filesystem::exists(p0)) {  
            // boost::filesystem::create_directories(p0);  
            // }
            // std::string joint_sor_path=file_path+"/jointCloud/"+std::to_string(idx)+"_"+std::to_string(idy)+"_"+std::to_string(idz)+"_"+std::to_string(ids)+"_"+std::to_string(grid_size)+".pcd";
            // pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZINormal>(joint_sor_path, *patch_cloud_g);
            // clock_t end=clock();
              // 停止计时
            auto end = std::chrono::high_resolution_clock::now();
              // 计算耗时
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // 输出结果
            std::cout<<"patchcloud:"<<patch_name<<" sor filter finish!cost time: "<< double(duration/1000.0) <<"s; pc num: "<<patch_cloud_filtered->size()<<std::endl;
            //std::cout<<"sorfilter finish!cost time: "<<double (end-start)/CLOCKS_PER_SEC<<"s; pc num: "<<patch_cloud_filtered->size()<<std::endl;
            // //smooth
            std::cout<<"mls start: "<<std::endl;
            // start=clock();
            auto start1 = std::chrono::high_resolution_clock::now();
            PointCloudINormalPtr patch_cloud_mls(new pcl::PointCloud<pcl::PointXYZINormal>);//patch_cloud_mls_filter(new pcl::PointCloud<pcl::PointXYZINormal>);
            MLS_smooth_PointINormalHandle(patch_cloud_g,patch_cloud_mls,2,0.1,true);//_filtered);
            // calStaticFilter(patch_cloud_mls, 50, 0.8, patch_cloud_mls_filter);
            // voxelFilter(patch_cloud_mls,patch_cloud_mls,0.01);
            // Upsample(patch_cloud_mls,10,0,0.3,2);
            // patch_cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
            boost::filesystem::path p1(file_path+"/smoothCloud");  
            if (!boost::filesystem::exists(p1)) {  
            boost::filesystem::create_directories(p1);  
            }
            std::string smooth_path=file_path+"/smoothCloud/"+std::to_string(idx)+"_"+std::to_string(idy)+"_"+std::to_string(idz)+"_"+std::to_string(ids)+"_"+std::to_string(grid_size)+".pcd";
            // pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZINormal>(smooth_path, *patch_cloud_f);
            pcl::PointCloud<PointXYZRGBIL>::Ptr patch_cloud_res= boost::make_shared<pcl::PointCloud<PointXYZRGBIL>>();
            // transformat_cloud(patch_cloud_filtered,patch_cloud_mls,patch_cloud_res);
            patch_cloud_mls.reset();//patch_cloud_mls_filter.reset();
            pcl::io::savePCDFileBinaryCompressed<PointXYZRGBIL>(smooth_path, *patch_cloud_res);
            // patch_cloud_res.reset();
            // end=clock();
            auto end1=std::chrono::high_resolution_clock::now();
            auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count();
            auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start).count();
            std::cout<<"patchcloud:"<<patch_name<<"mls finish!cost time: "<<double(duration1/1000.0) <<"s ;pc num: "<<patch_cloud_res->size()<<std::endl;
            std::cout<<"sigle patch cost total time: "<<double(duration2/1000.0) <<"s"<<std::endl;
            patch_cloud_res.reset();
        }
        // clock_t end_p=clock();
        auto end_t = std::chrono::high_resolution_clock::now();
        auto duration_p = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
        std::cout<<"sigle thread cost total time: "<<double(duration_p/1000.0)<<"s"<<std::endl;
        // std::cout<<"sigle thread cost total time: "<<double(end_p-start_t)/CLOCKS_PER_SEC<<"s"<<std::endl;
    }

void SaveCallback(){
    std::cout<<"Start process and save!============>"<<std::endl;
    LogInfo("Start process and save!============>\n");
    const int bar_width = 40;
     // 输出进度条
    int leftPadding = 60 ;
    int buffer_size =0;
    int pc_size =0;
    while (true) {
        // std::unique_lock<std::mutex> lock(mtx_buffer);
        mtx_buffer.lock();
        if (save_buffer_pcs.empty()&&buffer_deal_finish){
            mtx_buffer.unlock(); 
            break;
        }
        else if(save_buffer_pcs.empty()&&!buffer_deal_finish)
        {
            mtx_buffer.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else{
            buffer_size = save_buffer_pcs.size();
            // ...保存数据...

            
            boost::shared_ptr<pcl::PointCloud<PointXYZRGBIL>> cur_pc = save_buffer_pcs.at(0);
            std::string pc_path = buffer_save_names.at(0);
            save_buffer_pcs.pop_front();
            buffer_save_names.pop_front();
            mtx_buffer.unlock();
            pc_size = cur_pc->size();
            auto start_p = std::chrono::high_resolution_clock::now();
            // std::cout<<"Start save pcd==============>"<<std::endl;
            pcl::io::savePCDFileBinaryCompressed(pc_path, *cur_pc);
            // buffer_cv.notify_one();
            cur_pc->clear();
            cur_pc.reset();
            save_patch++;
            auto end_p = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed6 = end_p-start_p ;
            // 使用ANSI转义序列将光标移动到行首
            std::cout << "\r";
            double progress_percent = double(save_patch * 100) /total_patch ;
            int bar_filled = (save_patch * (bar_width-1)) / total_patch;
            std::cout << "Process and Save: [";
            // 输出进度条中的填充部分
            for (int i = 0; i < bar_filled; ++i) std::cout << '=';
            std::cout << '>';
            for (int i = bar_filled; i < bar_width-1; ++i) std::cout << ' ';
            // 输出进度百分比
            std::cout << "] " << progress_percent << "%"<<"("<<save_patch<<"/"<<total_patch<<")"<<" buf_size:"<<buffer_size<<" pc_size:"<<pc_size<<" Cost:"<<elapsed6.count()<<"s"<<"  ";
            std::cout.flush();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    LogInfo("Save PointCloud thread finish! Total save %d patch s!\n",save_patch);
    // return;
}


void Load_MappingCallback(std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>>spatial_map,std::string output_path)
{
    auto start_l = std::chrono::high_resolution_clock::now();
    LogInfo(" Load and Mapping PointCloud Thread start!!!!================>\n");
    Eigen::Matrix4d poseMat =Eigen::Matrix4d::Identity();
    for(const auto& pair:spatial_map)
    {
        std::vector<VertexData>vertexs=pair.second;
        XYZID<> index = pair.first;
        int64_t idx,idy,idz,ids;
        idx =index.xid;idy=index.yid;idz=index.zid;ids=index.sid;
        std::string cloud_patch_name =output_path+"/"+std::to_string(idx)+"_"+std::to_string(idy)+"_"+std::to_string(idz)+"_"+std::to_string(ids)+"_"+std::to_string(grid_size)+".pcd";
        double dbOffsetCX,dbOffsetCY;
        if (use2000)
        {
            ConvertGaosito2000(gauss_x0, gauss_y0,0.0,dbOffsetCX,dbOffsetCY);
        }
        boost::shared_ptr<PointCloudXYZIN> patch_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());//patch_cloud_ng(new pcl::PointCloud<pcl::PointXYZINormal>());
        auto start = std::chrono::high_resolution_clock::now();
        for(int j=0;j<vertexs.size();++j)
        {
            const std::string cloudpath=vertexs[j].CloudPath;
            pcl::io::loadPCDFile<pcl::PointXYZINormal>(cloudpath, *current_frame_cloud);
            // splitground_pc(current_frame_cloud,cloud_g,cloud_ng);
            if(use_self_point_filter)
            {
                RemoveEgoPoint(current_frame_cloud,current_frame_cloud_new,EgoRange);
                // RemoveEgoPoint(cloud_ng,current_frame_cloud_ng_new,EgoRange);
                // RemoveEgoPoint(cloud_g,current_frame_cloud_g_new,EgoRange);
                if(j%300==0)

                    pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZINormal>(output_path+"/"+vertexs[j].Bagname+"single"+vertexs[j].Timestamp+".pcd", *current_frame_cloud_new);
            }
            else{
                *current_frame_cloud_new = *current_frame_cloud;
                // *current_frame_cloud_g_new = *cloud_g;
                // *current_frame_cloud_ng_new = *cloud_ng;
            }
            poseMat(0,3)= vertexs[j].Tx;poseMat(1,3)=vertexs[j].Ty;poseMat(2,3)=vertexs[j].Tz;
            Eigen::Quaternion quat = RPYDatatoQuatData(vertexs[j].Rx,vertexs[j].Ry,vertexs[j].Rz);
            Eigen::Matrix3d rotmat =quat.normalized().matrix();
            poseMat.block<3, 3>(0, 0) = rotmat;
            if(use2000)
            {
                ConvertPgoDataAsCgcs2000(poseMat,gauss_x0, gauss_y0,dbOffsetCX,dbOffsetCY);
            }

            // *patch_cloud_ng+=*TransformPointCloud(current_frame_cloud_ng_new,poseMat);
            *patch_cloud+=*TransformPointCloud(current_frame_cloud_new,poseMat);

            current_frame_cloud->clear();
            current_frame_cloud_new->clear();
            // current_frame_cloud_g_new->clear();
        }
        auto end = std::chrono::high_resolution_clock::now();      
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        while(true)
        {
            {
                std::lock_guard<std::mutex> lock(load_mutex);
                if (load_buffer_pcs.size() < BUFFER_SIZE_LIMIT) {
                    load_buffer_pcs.push_back(patch_cloud);
                    buffer_patch_names.push_back(cloud_patch_name);
                    buffer_save_names.push_back(cloud_patch_name);
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        LogInfo(" Load and Mapping Patch cloud %sfinish ! Total cost: %.4f \n",cloud_patch_name.substr(cloud_patch_name.rfind("/")).c_str(),double(duration/1000.0));
        // std::cout<<"Load and Mapping Patch cloud"<<cloud_patch_name.substr(cloud_patch_name.rfind("/"))<<"finish ! Total cost:"<<double(duration/1000.0)<<std::endl;
    }
    buffer_load_finish=true;
    // buffer_cv.notify_all();
    auto end_l = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_l -start_l;
    LogInfo(" Load and Mapping PointCloud Thread finish! Total cost: %.4f  ! Total load num = %d\n",elapsed.count(),spatial_map.size());
}

// 处理函数
void ProcessCallback() {
    int buffer_size =0;
    LogInfo("Process Thread start!!!!!!!!!!!!!!!!!============>\n");
    PointCloudINormalPtr patch_cloud_filtered(new pcl::PointCloud<pcl::PointXYZINormal>()),patch_cloud_mls(new pcl::PointCloud<pcl::PointXYZINormal>());
    while (true) {
        mtx_process1.lock();
        if (buffer_load_finish && load_buffer_pcs.empty()) {
            mtx_process1.unlock();
            break; // 如果生产者完成且缓冲区为空，则退出循环
        }
        else if (!buffer_load_finish&&load_buffer_pcs.empty())
        {
            mtx_process1.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else if(!load_buffer_pcs.empty()&&save_buffer_pcs.size()<BUFFER_SIZE_LIMIT)
        {
            // 从缓冲区取出数据
            boost::shared_ptr<PointCloudXYZIN> patch_cloud = load_buffer_pcs.at(0);

            std::string patch_out_name = buffer_patch_names.at(0);
            buffer_size = load_buffer_pcs.size();
            load_buffer_pcs.pop_front();
            buffer_patch_names.pop_front();
            // 通知生产者缓冲区有空位
            mtx_process1.unlock();
            // buffer_cv.notify_one();
            //
            LogInfo("Current process's buffer size = %d\n",buffer_size);
            // std::cout<<"Current process's pc buffer size ="<<buffer_size<<std::endl;
            // save joint cloud
            if(save_joint_flage)
            {
                std::string save_joint_path = patch_out_name;
                size_t pos = patch_out_name.find(outname);
                if (pos!=std::string::npos)
                {
                    save_joint_path.replace(pos,strlen(outname.c_str()),jointname);
                }
                pcl::io::savePCDFileBinaryCompressed(save_joint_path,*patch_cloud);
            }
            // 处理数据...
            std::string patch_name = patch_out_name.substr(patch_out_name.rfind("/")+1,patch_out_name.find(".")-patch_out_name.rfind("/")-1);
            //std::cout<<"patchcloud:"<< patch_name <<"; Sorfilter start, pc num:"<<patch_cloud->size()<<"========>"<<std::endl;
            LogInfo("Patchcloud name = %s; patch pc num:%d; .Sorfilter start! ==============>",patch_name.c_str(),patch_cloud->size());
            auto start = std::chrono::high_resolution_clock::now();
            // lock.lock();
            calStaticFilter(patch_cloud, K, K_threshold, patch_cloud_filtered,available_opm_num);
            // *patch_cloud_filtered =* patch_cloud_nground;
            // patch_cloud_filtered= GaussianFilter_PointINormalHandle(patch_cloud_f,4,0.1);
            auto end = std::chrono::high_resolution_clock::now();
                // 计算耗时
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // 输出结果
            patch_cloud.reset();
            //std::cout<<"Sor filter finish!cost time: "<< double(duration/1000.0) <<"s; pc num: "<<patch_cloud_filtered->size()<<"! Start Voxel==========>"<<std::endl;
            LogInfo("Sor filter finish!cost time: %.6f s; Patch pc num be changed:%d ! Start Voxel Filter==========>\n",double(duration/1000.0),patch_cloud_filtered->size());
            // voxelFilter(patch_cloud_ground,patch_cloud_ground,leaf_size);
            voxelFilter_xs(patch_cloud_filtered,leaf_size);
            auto end_v = std::chrono::high_resolution_clock::now();
            auto duration_v = std::chrono::duration_cast<std::chrono::milliseconds>(end_v -end).count();
            //std::cout<<"Voxel Filter finish ! Cost time "<<double(duration_v/1000.0) <<"s! After voxel filter patch pc num :"<<patch_cloud_filtered->size()<<std::endl;
            //std::cout<<"MLS start! ======================> "<<std::endl;
            LogInfo("Voxel Filter finish ! Cost time = %.6f s! After voxel filter patch pc num : %d\n",double(duration_v/1000.0),patch_cloud_filtered->size());
            LogInfo("MLS start!==========================>\n");
            auto start1 = std::chrono::high_resolution_clock::now();
            MLS_smooth_PointINormalHandle(patch_cloud_filtered,patch_cloud_mls,available_opm_num,Ploynomialorder,search_radius,true);//_filtered);
            // *patch_cloud_mls= *patch_cloud_ground;
            // calStaticFilter(patch_cloud_mls, 50, 0.8, patch_cloud_mls_filter);
            patch_cloud_filtered->clear();
            // Upsample(patch_cloud_mls,10,0,0.3,2);
            // patch_cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
            auto start2 =std::chrono::high_resolution_clock::now();
            auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(start2 - start1).count();
            //std::cout<<"MLS finish!cost time: "<<double(duration1/1000.0)<<"s;After MLS patch pc num =" <<patch_cloud_mls->size()<<";!Start transformat=========>"<<std::endl;
            LogInfo("MLS finish!Cost time: %.6f  s ;After MLS patch pc num = %d!\n",double(duration1/1000.0),patch_cloud_mls->size());
            LogInfo("Start transformat=========>\n");
            boost::shared_ptr<pcl::PointCloud<PointXYZRGBIL>> patch_cloud_res= boost::make_shared<pcl::PointCloud<PointXYZRGBIL>>();
            transformat_cloud(patch_cloud_mls,patch_cloud_res);
            patch_cloud_mls->clear();
            auto end1=std::chrono::high_resolution_clock::now();
            auto duration_t = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start2).count();
            //std::cout<<"Tranformat finish! Total result pc num: "<<patch_cloud_res->size()<<std::endl;
            LogInfo("Tranformat finish!Cost time:%.6f;Total result pc num: %d\n",double(duration_t/1000.0),patch_cloud_res->size());
            // LogInfo("Start save pcd===============>\n");
            mtx_process2.lock();
            save_buffer_pcs.push_back(patch_cloud_res);
            mtx_process2.unlock();
            // pcl::io::savePCDFileBinaryCompressed<PointXYZRGBIL>(patch_out_name, *patch_cloud_res);
            deal_patch++;
            auto end2=std::chrono::high_resolution_clock::now();
            // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - end1).count();
            auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start).count();
            // std::cout<<"Save pointcloud finish, cost:"<<double(duration2/1000.0)<<"s"<<std::endl;
            //std::cout<<"Sigle patch cost total time: "<<double(duration3/1000.0) <<"s;"<<"Deal Process:================>"<<deal_patch<<"/"<<total_patch<<std::endl;
            // LogInfo("Save pointcloud finish! Cost: %.6f s\n",double(duration2/1000.0));
            LogInfo("Sigle patch cost total time: %.6f s !    Deal Process: %d/%d\n",double(duration3/1000.0),deal_patch,total_patch);
        }
        else{
            mtx_process1.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    buffer_deal_finish=true;
    // buffer_cv.notify_all();
    LogInfo("Process and Save PointCloud thread finish! total deal %d patchs!======================>\n",deal_patch);
    return;
}


int main(int argc, char** argv)
 {
    std::string input_path="";
    std::string output_path ="";
    std::string log_path="";
    std::string params_path ="";

    // 遍历命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        // 检查是否为输入目录参数
        if (arg == "-i" && i + 1 < argc) {
            input_path = argv[++i]; 
        }
        // 检查是否为输出目录参数
        else if (arg == "-o" && i + 1 < argc) {
            output_path = argv[++i];
        }
        else if (arg == "-inname" && i + 1 < argc) {
            inname = argv[++i];
        }
        else if (arg == "-outname" && i + 1 < argc) {
            outname = argv[++i];
        }
        else if (arg == "-log" && i + 1 < argc) {
            log_path = argv[++i]; 
        }
        else if (arg == "-config" && i + 1 < argc) {
            params_path = argv[++i]; 
        }
        else if ((arg =="-v"||arg=="-V")&&i < argc)
        {
            printf("%s %s.%s.%s.%s %s %s released.\n", argv[0], CC_APP_XXX_VERSION_NAME, CC_APP_XXX_MAJOR_VERSION, CC_APP_XXX_MINOR_VERSION, CC_APP_XXX_PATCH_VERSION, __DATE__, __TIME__);
            return 0;
        }
        else if ((arg =="-h"||arg=="-H")&&i < argc)
        {
                printf("Usage: %s [-v/-V]show version [-h/H] help \n", argv[0]);
                printf("-i input_dir -o output_dir -log log_dir -config programs params file path -inname input dictory's name -outname output dictory's name \n");
                printf("input:need a pgodata path,output path, programs params file path , log dir 、 in and out dictory name\n");
                return 0;
        }
        else {
            std::cerr << "Invalid argument: " << arg << std::endl;
            return 1; // 非法参数，退出程序
        }
    }
    // 检查是否提供了必要的参数
    if (input_path.empty() || output_path.empty()||log_path.empty()||params_path.empty() || output_path.empty()||inname.empty()||outname.empty()) {
        std::cerr << "Usage: " << argv[0] << " -i input_dir -o output_dir -log log_dir -config programs params file path" << std::endl;
        return 1;
    }

    initLog(log_path);

    std::unordered_map<std::string, std::string> configMap;
    configMap = read_params(params_path);

    std::string thread_num= configMap["thread_num"];
    std::string process_num = configMap["process_num"];
    std::string use_2000 = configMap["use_2000"];

    grid_size =std::stoi(configMap["patch_size"]);
    image_link_radius = std::stoi(configMap["image_link_radius"]);
    int numThreads =std::stoi(thread_num);//std::thread::hardware_concurrency()/2; // 获取可用的线程数
    int numProcess =std::stoi(process_num);
    BUFFER_SIZE_LIMIT = std::stoi(configMap["BUFFER_SIZE_LIMIT"]);

    std::vector<double>point;
    point.push_back(std::stod(configMap["guass_point_x"]));
    point.push_back(std::stod(configMap["guass_point_y"]));
    use_self_point_filter = configMap["use_self_point_filter"]=="true"||configMap["use_self_point_filter"]=="True"?true:false;
    if(use_self_point_filter)
    {
        EgoRange.push_back(std::stod(configMap["self_x_min"]));
        EgoRange.push_back(std::stod(configMap["self_x_max"]));
        EgoRange.push_back(std::stod(configMap["self_y_min"]));
        EgoRange.push_back(std::stod(configMap["self_y_max"]));
        EgoRange.push_back(std::stod(configMap["self_z_min"]));
        EgoRange.push_back(std::stod(configMap["self_z_max"])); 
    }  
    use2000 = use_2000=="true"||use_2000=="Ture"? true:false;
    gauss_x0 = point[0];
    gauss_y0 = point[1];
    // std::cout<<use2000<<"x:"<<gauss_x0<<"y:"<<gauss_y0<<std::endl;
    K = std::stoi(configMap["K"]);
    K_threshold =std::stof(configMap["K_threshold"]);
    Ploynomialorder = std::stoi(configMap["Ploynomialorder"]);
    search_radius =std::stof(configMap["search_radius"]);
    leaf_size = std::stod(configMap["leaf_size"]);
    std::string save_joint_f = configMap["save_joint_flage"];
    save_joint_flage = save_joint_f=="true"|| save_joint_f=="True"?true:false;

    LogInfo("User's Pgodata file path is: %s\n",input_path.c_str());
    LogInfo("User's output path name is %s\n",output_path.c_str());
    LogInfo("User's input dict name is %s ,and output dict name is %s\n",inname.c_str(),outname.c_str());
    LogInfo("Use Params: patch size = %d; image_link_radius = %d; thread number = %d;parall process thread number = %d;\n ",grid_size,image_link_radius,numThreads,numProcess);
    LogInfo("            use_2000 = %s; guass point = [%.9f,%.9f];\n ",use_2000.c_str(),gauss_x0,gauss_y0);
    LogInfo("            use_ego_point_filter= %s, ego range = [%s ,%s ,%s ,%s ,%s ,%s]\n",configMap["use_self_point_filter"].c_str(),configMap["self_x_min"].c_str(),
    configMap["self_x_max"].c_str(),configMap["self_y_min"].c_str(),configMap["self_y_max"].c_str(),configMap["self_z_min"].c_str(),configMap["self_z_max"].c_str());
    LogInfo("            SOR:need K = %d ,K_threshold =%.2f; MLS need:Ploynomialorder = %d ,search_radius =%.2f;\n ",K,K_threshold,Ploynomialorder,search_radius);
    LogInfo("            Voxel filter:need leaf_size = %.6f.\n",leaf_size);
    // std::cout<<"threads available:"<<std::thread::hardware_concurrency()<<std::endl;
    LogInfo("Thread available is %d  now\n!",std::thread::hardware_concurrency());
    LogInfo("User's input thread_num is %d,and process(omp) num is %d !\n",numThreads,numProcess);
    LogInfo("User choose BUFFER_SIZE_LIMIT =%d\n",BUFFER_SIZE_LIMIT);
    LogInfo("User choose wether save joint falge = %s! \n",save_joint_f.c_str());
    if(BUFFER_SIZE_LIMIT<numThreads)
    {
        LogInfo("Thread nums must be smaller than BUFFER_SIZE_LIMIT!!!!!!!!!!!!!!!!!!!!!!!!!\n ");
        return 1;
    }
    if(numThreads*numProcess>std::thread::hardware_concurrency())
    {
        std::cout<< "the thread_num*process_num must be greater than thread available!!!!!!!!!"<<std::endl;
        LogError("The thread_num*process_num must be greater than thread available!!!!!!!!!!!!\n");
        return 0;
    }
    boost::filesystem::path p1(output_path+"/"+outname);  
    if (!boost::filesystem::exists(p1)) {  
    boost::filesystem::create_directories(p1);  
    LogInfo("Create sucessfuly final output path: %s/%s !\n",output_path.c_str(),outname.c_str());
    }
    std::string cloud_output_path = output_path+"/"+outname;

    // 清空 `merge_indexs` 目录
    boost::filesystem::path p2(cloud_output_path+"/merge_indexs");
    if (boost::filesystem::exists(p2)) {
        boost::filesystem::remove_all(p2);  // 递归删除 `merge_indexs` 目录中的所有内容
    }
    boost::filesystem::create_directory(p2);
    std::string merge_index_info_path = cloud_output_path+"/merge_indexs";
    std::vector<VertexData> vertexDataList;std::vector<double> range;
    clock_t load_pgo_s=clock();
    bool load_pgoinfo = LoadPgodataInfo(input_path,vertexDataList,range);
    clock_t load_pgo_e=clock();
    // std::cout<<"load pgodata cost: "<<double(load_pgo_e-load_pgo_s)/CLOCKS_PER_SEC<<"s"<<std::endl;
    LogInfo("load pgodata cost: %.6f\n",double(load_pgo_e-load_pgo_s)/CLOCKS_PER_SEC);
    if (load_pgoinfo)
    {
        std::cout<<"Load Pgodata infos sucesss! Total load "<<vertexDataList.size()<<"frame info!"<<std::endl;
        LogInfo("Load Pgodata infos sucesss! Total load %d frame info!\n",vertexDataList.size());
    }

    std::cout<<"range list:  {min_x="<<range[0]<<" max_x="<<range[1]<<" min_y="<<range[2]<<" max_y="<<range[3]
            <<" min_z="<<range[4]<<" max_z="<<range[5]<<"}"<<std::endl;
 

    std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>> spatial_map = build_spatial_index(vertexDataList, grid_size);
    // std::cout<<"Before adjust map,spatailmap size:"<<spatial_map.size()<<std::endl;
    if (!spatial_map.size()>0)
    {
        std::cout<<" Build spatial index not successfuly!!!!!!!!!!"<<std::endl;
        LogError("Build spatial index not successfuly!!!!!!!!!!\n");
    }
    LogInfo("Start Adjust spatial map,spatailmap size:%d\n",spatial_map.size());
    std::cout<<"Start adjust map,spatailmap size:"<<spatial_map.size()<<std::endl;
    Adjustmap(spatial_map,merge_index_info_path);
    std::cout<<"After adjust map,spatailmap size:"<<spatial_map.size()<<std::endl;
    LogInfo("After Adjust spatial map,spatailmap size:%d\n",spatial_map.size());
    std::unordered_map<int,VertexData> id2vertex_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pose = Pose2pcd(vertexDataList,id2vertex_map);

    boost::filesystem::path p3(output_path+"/PatchInfo");  
    if (!boost::filesystem::exists(p3)){  
    boost::filesystem::create_directories(p3);  
    LogInfo("Create sucessfuly final patch info output path: %s/%s !\n",output_path.c_str(),outname.c_str());
    }
    std::string info_output_path = output_path+"/PatchInfo";
    if(save_joint_flage)
    {
        std::string joint_output_path = output_path+"/"+jointname;
        boost::filesystem::path p4(output_path+"/"+jointname);  
        if (!boost::filesystem::exists(p4)){  
        boost::filesystem::create_directories(p4);  
        LogInfo("Create sucessfuly joint patch output path: %s/%s !\n",output_path.c_str(),jointname.c_str());
        }
        LogInfo("if choose true the output path is %s\n",joint_output_path.c_str());
    }
    LogInfo("Start to save patch info file! Include: image/cloud/timestamp/bagname info!\n");
    ImageLinkIndex(spatial_map,cloud_pose,id2vertex_map,image_link_radius,grid_size,info_output_path);
    LogInfo("Save patch info file finish!\n");
    // adjustmap(spatial_map,file_path);
    int count=spatial_map.size();
    total_patch = count;
    numThreads=std::min(std::min(count,int(std::thread::hardware_concurrency())),numThreads);
    std::cout<< "actual tile nums : " <<count<<" and threads num:"<<numThreads<<std::endl;
    available_opm_num = numProcess;//std::thread::hardware_concurrency()/(numProcess*numThreads)-1>1?std::thread::hardware_concurrency()/(numProcess*numThreads)-1:1;
    // std::cout<<"omp__num is"<<available_opm_num<<std::endl;
    LogInfo("Actual maptile number is:%d .And actually thread num is %d by used!\n",count,numThreads);

    std::thread loadThread([&spatial_map,&cloud_output_path](){
        try {
                Load_MappingCallback(spatial_map,cloud_output_path);
        } catch (const std::exception& e) {
            // std::cerr << "thread1 exception: " << e.what() << std::endl;
            LogError("PointCloud load and mapping thread exception: %s\n",e.what());
        }
    });
    // std::thread processThread([](){
    //     try {
    //            ProcessCallback();
    //     } catch (const std::exception& e) {
    //         // std::cerr << "thread1 exception: " << e.what() << std::endl;
    //         LogError("PointCloud process thread exception: %s\n",e.what());
    //     }
    // });
    // 启动三个处理线程
    std::thread processThreads[numThreads];
    for (int i = 0; i < numThreads; ++i) {
        processThreads[i] = std::thread(ProcessCallback);
    }

    std::thread saveThread(SaveCallback);
    auto endc = std::chrono::high_resolution_clock::now();
    loadThread.join();
    // 等待所有处理线程完成
    for (int i = 0; i < numThreads; ++i) {
        processThreads[i].join();
    }
    saveThread.join();
    // processThread.join();
    auto ends = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed5 = ends -endc;
    std::cout<<std::endl;

    std::cout<<"Deal total cost time:"<<elapsed5.count() <<"s; single patch cloud averge cost : "<<elapsed5.count()/count<<std::endl;
    LogInfo("patch deal total cost time: %.4f s; single frame averge cost: %.5f s;\n",elapsed5.count(),elapsed5.count()/count);

    return 0;
}




