#ifndef BUILDSPATIALMAP_H
#define BUILDSPATIALMAP_H
#pragma once

#include <iostream>
#include <vector>
#include<unordered_map>
#include<map>
#include <boost/filesystem.hpp>
#include <math.h>
#include<ctime>
#include<cstdlib>
#include <Eigen/Core>
#include<deque>
#include<set>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>///n
#include <pcl/point_cloud.h>

#include"xsproto_common_common.h"


// 定义 XYZID 坐标的结构体
template <typename T= int64_t>
struct XYZID {
    T xid, yid, zid,sid;
    
    // 重载哈希函数
    size_t operator()(const XYZID& xyzid) const {
        // 使用简单的哈希算法，将 x、y、z 合并为一个哈希值
        return std::hash<T>()(xyzid.xid) ^ std::hash<T>()(xyzid.yid) ^ std::hash<T>()(xyzid.zid)^ std::hash<T>()(xyzid.sid);
    }
    // 重载比较运算符
    bool operator==(const XYZID& other) const {
        return xid == other.xid && yid == other.yid && zid == other.zid&& sid == other.sid;
    }
};

// 定义一个结构体来存储所需的字段
struct VertexData {
    double Tx, Ty, Tz, Rx, Ry, Rz;
    int id;
    std::string Timestamp,Bagname,CloudPath;
};

std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>> buildSpatialMap(std::map<std::string ,std::string> filenames,
    std::vector<std::pair<std::string,Eigen::Matrix4d>> poseMatVec,int grid_size);
void dfs(std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>>& spatial_map,const XYZID<>&key,
        std::string file_path,std::vector<std::string>&value,std::vector<XYZID<>>& neighbor_indexs,std::vector<XYZID<>>& keys_to_erase);
void split(std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>>& sub_spatial_map,const XYZID<>&key,
        std::vector<std::string>&value);
void adjustmap(std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>>& spatial_map,std::string file_path );
std::vector<std::vector<std::vector<std::vector<int>>>> tile_operate(std::vector<Eigen::Matrix4d> poseMatVec,
    double min_x,double max_x,double min_y,double max_y,double min_z,double max_z, int grid_size);

std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>> build_spatial_index(std::vector<VertexData> vertexdata_list,int grind_size);
pcl::PointCloud<pcl::PointXYZI>::Ptr Pose2pcd(std::vector<VertexData> vertexs,std::unordered_map<int,VertexData>&id2vertex_map);
void ImageLinkIndex(std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>> spatialmap,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    std::unordered_map<int,VertexData>id2vertex_map,float link_radius,int grid_size,std::string output_path);
void Adjustmap(std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>>& spatial_map,std::string file_path );

void DFS(std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>>& spatial_map,const XYZID<>&key,std::string file_path,
        std::vector<VertexData>&value,std::vector<XYZID<>>& neighbor_indexs,std::vector<XYZID<>>& keys_to_erase);

#endif