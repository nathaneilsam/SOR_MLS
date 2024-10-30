#include"buildSpatialMap.h"

std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>> buildSpatialMap(std::map<std::string ,std::string> filenames,
    std::vector<std::pair<std::string,Eigen::Matrix4d>> poseMatVec,int grid_size)
 {
    std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>> spatial_map;
    for(int i=0;i<poseMatVec.size();++i)
    {
        double x=poseMatVec[i].second(0,3),y=poseMatVec[i].second(1,3),z=poseMatVec[i].second(2,3);
        double vidx = x / grid_size;
        double vidy = y / grid_size;
        double vidz = z / grid_size;
        if (vidx < 0) vidx -= 1;
        if (vidy < 0) vidy -= 1;
        if (vidz < 0) vidz -= 1;
        XYZID<> index = {static_cast<int64_t>(vidx), static_cast<int64_t>(vidy), static_cast<int64_t>(vidz),static_cast<int64_t>(0)};
        
        // 检查索引是否已存在于 spatial_map 中
        if (spatial_map.find(index) != spatial_map.end()) {
            // 如果已存在，将当前索引添加到对应的 vector 中
            std::string id=poseMatVec[i].first;
            if(filenames.find(id)!=filenames.end())
            {
                spatial_map[index].push_back(id);
            }
        } else {
            // 如果不存在，创建一个新的 vector 并将当前索引存入
            std::string id=poseMatVec[i].first;
            if(filenames.find(id)!=filenames.end())
            {
                spatial_map[index] = {id};
            }
                
        }
    }
    return spatial_map;
 }
void dfs(std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>>& spatial_map,const XYZID<>&key,
        std::string file_path,std::vector<std::string>&value,std::vector<XYZID<>>& neighbor_indexs,std::vector<XYZID<>>& keys_to_erase)
 {
    if(value.size()>=50||value.size()==0) return;
    int idx=key.xid,idy=key.yid,idz=key.zid,ids=key.sid;
    std::vector<int> nez={1,-1, 2,-2};
    for(int n=0;n<nez.size()-1&&value.size()>0;++n)
    {
        XYZID<> nei_index = {static_cast<int64_t>(idx), static_cast<int64_t>(idy), static_cast<int64_t>(idz+nez[n])};
        auto neighbor_it = spatial_map.find(nei_index);
        if (neighbor_it != spatial_map.end())
        {
            std::vector<std::string>& neighbor_value=spatial_map.find({nei_index})->second;//spatial_map[nei_index];
            if(neighbor_value.size()>0)
            {
                value.insert(value.end(),neighbor_value.begin(),neighbor_value.end());
                keys_to_erase.push_back(nei_index);
                neighbor_indexs.push_back(nei_index);
                neighbor_value.clear();
                if(value.size()>=50)
                {
                    // boost::filesystem::path p1(file_path+"/smoothCloud/merge_indexs");  
                    // if (!boost::filesystem::exists(p1)) {  
                    // boost::filesystem::create_directories(p1);  
                    // }
                    boost::filesystem::path p1(file_path + "/smoothCloud");
                    if (!boost::filesystem::exists(p1)) {
                        boost::filesystem::create_directory(p1);
                    }
                    // 清空 `merge_indexs` 目录
                    boost::filesystem::path p2(file_path + "/smoothCloud/merge_indexs");
                    if (boost::filesystem::exists(p2)) {
                        boost::filesystem::remove_all(p2);  // 递归删除 `merge_indexs` 目录中的所有内容
                    }
                    boost::filesystem::create_directory(p2);
                    std::string merge_indece=file_path+"/smoothCloud/merge_indexs/"+std::to_string(idx)+"_"+std::to_string(idy)+"_"+std::to_string(idz)+".txt";
                    std::ofstream outfile(merge_indece, std::ios_base::app);
                    if (outfile.is_open()) {
                        for(int i=0;i<neighbor_indexs.size();++i)
                        {
                            outfile << neighbor_indexs[i].xid << " " << neighbor_indexs[i].yid << " " << neighbor_indexs[i].zid << std::endl; 
                        }
                        outfile.close();
                        std::cout << "File created: " << merge_indece << std::endl;
                    } else {
                        std::cerr << "Unable to open file: " << merge_indece << std::endl;
                    }
                    neighbor_indexs.clear();
                    return;
                }
            }
        }
    }
    std::vector<int> nei = {1, 0, -1, 0, 1};//, 1, -1, -1, 1};
    // bool found_neighbor = false;
    for(int n=0;n<nei.size()-1;++n)
    {
        int ex=nei[n],ey=nei[n+1];
        XYZID<> nei_index = {static_cast<int64_t>(idx+ex), static_cast<int64_t>(idy+ey), static_cast<int64_t>(idz)};
        auto neighbor_it = spatial_map.find(nei_index);
        if (neighbor_it != spatial_map.end())
        {
            std::vector<std::string>& neighbor_value=spatial_map.find({nei_index})->second;//spatial_map[nei_index];
            if(neighbor_value.size()>0)
            {
                neighbor_value.insert(neighbor_value.end(),value.begin(),value.end());
                // found_neighbor = true;
                neighbor_indexs.push_back(key);
                keys_to_erase.push_back(key); // 添加要删除的键     
                value.clear();
                if(neighbor_value.size()>=50)
                {
                    boost::filesystem::path p1(file_path+"/smoothCloud/merge_indexs");  
                    if (!boost::filesystem::exists(p1)) {  
                    boost::filesystem::create_directories(p1);  
                    }
                    std::string merge_indece=file_path+"/smoothCloud/merge_indexs/"+std::to_string(idx+ex)+"_"+std::to_string(idy+ey)+"_"+std::to_string(idz)+".txt";
                    std::ofstream outfile(merge_indece, std::ios_base::app);
                    if (outfile.is_open()) {
                        for(int i=0;i<neighbor_indexs.size();++i)
                        {
                            outfile << neighbor_indexs[i].xid << " " << neighbor_indexs[i].yid << " " << neighbor_indexs[i].zid << std::endl; 
                        }
                        outfile.close();
                        std::cout << "File created: " << merge_indece << std::endl;
                    } else {
                        std::cerr << "Unable to open file: " << merge_indece << std::endl;
                    }
                    neighbor_indexs.clear();
                    return;
                }
                else 
                {
                    // neighbor_indexs.push_back(key);
                    dfs(spatial_map,nei_index,file_path,neighbor_value,neighbor_indexs,keys_to_erase);
                    return;    
                }                
            }
            else{
                if(n==nei.size()-1)
                {
                    boost::filesystem::path p1(file_path+"/smoothCloud/merge_indexs");  
                    if (!boost::filesystem::exists(p1)) {  
                    boost::filesystem::create_directories(p1);  
                    }
                    std::string merge_indece=file_path+"/smoothCloud/merge_indexs/"+std::to_string(idx+ex)+"_"+std::to_string(idy+ey)+"_"+std::to_string(idz)+".txt";
                    std::ofstream outfile(merge_indece, std::ios_base::app);
                    if (outfile.is_open()) {
                        for(int i=0;i<neighbor_indexs.size();++i)
                        {
                            outfile << neighbor_indexs[i].xid << " " << neighbor_indexs[i].yid << " " << neighbor_indexs[i].zid << std::endl; 
                        }
                        outfile.close();
                        std::cout << "File created: " << merge_indece << std::endl;
                    } else {
                        std::cerr << "Unable to open file: " << merge_indece << std::endl;
                    }
                    neighbor_indexs.clear();
                    return;
                }
                else continue;
            }
                   
        }
                 
    }
 }

void split(std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>>& sub_spatial_map,const XYZID<>&key,
        std::vector<std::string>&value)
{   
    assert(value.size()>100 && "split's frame size must be more than 100");
    std::cout<<"start split"<<std::endl;
    std::sort(value.begin(), value.end(),[](std::string value1,std::string value2){
      return  std::stoll(value1)< std::stoll(value2);});
    int submap_count=std::floor(value.size()/50);
    int submap_size=int(value.size()/submap_count);
    std::cout<<"split num and submap size:"<<submap_count<<" "<<submap_size<<std::endl;
    for(int i=1;i<submap_count;++i)
    {
        std::vector<std::string>subvalue;
        // subvalue.resize(submap_size+submap_count); // 或者使用其他初始化方法
        XYZID<> subkey={static_cast<int64_t>(key.xid), static_cast<int64_t>(key.yid), static_cast<int64_t>(key.zid),static_cast<int64_t>(i)};
        if(i!=submap_count-1)
        {
            subvalue.assign(value.begin()+i*submap_size,value.begin()+(i+1)*submap_size);
        }
        else
            subvalue.assign(value.begin()+i*submap_size,value.end());
        sub_spatial_map[subkey]=subvalue;

    }
    value.resize(submap_size);
}

void adjustmap(std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>>& spatial_map,std::string file_path )
{
    std::vector<XYZID<>> neighbor_indexs;//保存合并的键
    std::vector<XYZID<>> keys_to_erase; // 保存要删除的键
    std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>>sub_spatial_map;// 保存要分裂的键
    for (auto it = spatial_map.begin(); it != spatial_map.end();++it ) 
    {
        const XYZID<>& key = it->first;
        std::vector<std::string>& value = it->second;
        if(value.size()>=50||value.size()==0) {continue;}
        else if(value.size()<50)
            dfs(spatial_map,key,file_path,value,neighbor_indexs,keys_to_erase);
    }
    // 在循环外部删除键
    for (const auto& key : keys_to_erase) {
        spatial_map.erase(key);
    }
    // for (auto it = spatial_map.begin(); it != spatial_map.end();++it )
    // {
    //     const XYZID<>& key = it->first;
    //     std::vector<std::string>& value = it->second;
    //     // if(value.size()>100)
    //     //     // split(sub_spatial_map,key,value);
    //     // else 
    //     //     continue;
    // } 
    //添加分裂的键值
    // for (auto it = sub_spatial_map.begin(); it != sub_spatial_map.end();++it )
    // {
    //     spatial_map.insert({it->first,it->second});
    // }
}

std::vector<std::vector<std::vector<std::vector<int>>>> tile_operate(std::vector<Eigen::Matrix4d> poseMatVec,
    double min_x,double max_x,double min_y,double max_y,double min_z,double max_z, int grid_size)
    {
        int row_num=std::ceil((max_x-min_x)/grid_size)+1,lis_num=std::ceil((max_y-min_y)/grid_size)+1,
        heigh_num=std::ceil((max_z-min_z)/grid_size)+1;
        std::vector<std::vector<std::vector<std::vector<int>>>>tilemap(row_num,std::vector<std::vector<std::vector<int>>>(lis_num,
        std::vector<std::vector<int>>(heigh_num,std::vector<int>())));
        for(int i=0;i<poseMatVec.size();++i)
        {
            double x=poseMatVec[i](0,3),y=poseMatVec[i](1,3),z=poseMatVec[i](2,3);
            int xid=int(x/grid_size),yid=int(y/grid_size),zid=int(z/grid_size);
            int lowxid=int(min_x/grid_size),lowyid=int(min_y/grid_size),lowzid=int(min_z/grid_size);
            int row=xid-lowxid,lis=yid-lowyid,heigh=zid-lowzid;
            tilemap[row][lis][heigh].push_back(i);

        }
        return tilemap;//,freqVec};
    }


std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>> build_spatial_index(std::vector<VertexData> vertexdata_list,int grid_size)
{
    std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>> spatial_map;
    for(int i=0;i<vertexdata_list.size();++i)
    {
        VertexData vertex = vertexdata_list[i];
        double x=vertex.Tx,y=vertex.Ty,z=vertex.Tz;
        double vidx = x / grid_size;
        double vidy = y / grid_size;
        double vidz = z / grid_size;
        if (vidx < 0) vidx -= 1;
        if (vidy < 0) vidy -= 1;
        if (vidz < 0) vidz -= 1;
        XYZID<> index = {static_cast<int64_t>(vidx), static_cast<int64_t>(vidy), static_cast<int64_t>(vidz),static_cast<int64_t>(0)};
        
        // 检查索引是否已存在于 spatial_map 中
        if (spatial_map.find(index) != spatial_map.end()) {
            // 如果已存在，将当前索引添加到对应的 vector 中
            spatial_map[index].push_back(vertex);

        } else {
            // 如果不存在，创建一个新的 vector 并将当前索引存入
            spatial_map[index] = {vertex};
                
        }
    }
    return spatial_map;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Pose2pcd(std::vector<VertexData> vertexs,std::unordered_map<int,VertexData>&id2vertex_map)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i=0;i<vertexs.size();++i)
    {
        VertexData vertex =vertexs[i];
        pcl::PointXYZI point;
        point.x = vertex.Tx;
        point.y = vertex.Ty;
        point.z = vertex.Tz;
        point.intensity = vertex.id;
        cloud->push_back(point);
        id2vertex_map[vertex.id] = vertex;
    }
    return cloud;
}

void ImageLinkIndex(std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>> spatialmap,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    std::unordered_map<int,VertexData>id2vertex_map,float link_radius,int grid_size,std::string output_path)
{
    // 填充点云数据
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    std::cout<<"Start to save patch info!!!====> Include:"<<std::endl;
    int count =0;
    for(const auto & pair:spatialmap)
    {
        XYZID<>  index = pair.first;
        int64_t idx,idy,idz,ids;
        idx =index.xid;idy=index.yid;idz=index.zid;ids=index.sid;
        std::string map_index=std::to_string(idx)+"_"+std::to_string(idy)+"_"+std::to_string(idz)+"_"+std::to_string(ids);
        std::string map_index_filename = output_path+"/"+std::to_string(idx)+"_"+std::to_string(idy)+"_"+std::to_string(idz)+"_"+std::to_string(ids)+"_"+std::to_string(grid_size)+".txt";
        std::vector<VertexData> vertexs = pair.second;
        std::set<int> vertex_ids;
        for(int i=0;i<vertexs.size();i=i+5)
        {
            VertexData vertex =vertexs[i];
            double x = vertex.Tx,y=vertex.Ty,z = vertex.Tz;
            vertex_ids.insert(vertex.id);
            pcl::PointXYZI search_point;
            search_point.x = x;
            search_point.y = y;
            search_point.z = z;
            std::vector<int> pointIdxRadiusSearch; // 存储找到的点的索引
            std::vector<float> pointRadiusSquaredDistance; // 存储找到的点与查询点之间的平方距离

            double radius = link_radius; // 设置搜索半径
            if (tree->radiusSearch(search_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                // 处理找到的点
                for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                {
                    const pcl::PointXYZI& point = cloud->points[pointIdxRadiusSearch[i]];
                    vertex_ids.insert(point.intensity);
                    // 使用点的坐标和距离进行操作
                }
            }
        }
        std::ofstream outFile;
        outFile.open(map_index_filename);
        if(!outFile.is_open())
        {
           LogError("Failed to open the patch info file: %s!\n",map_index_filename.c_str());
            // std::cerr<<"Failed to open the patch info file:"<<map_index_filename<<std::endl;
        }
        // LogInfo("patch name: %s ;patch cloud num: %d ;After R search ,cloud frame num: %d \n",map_index.c_str(),vertexs.size(),vertex_ids.size());
        outFile<<"patch name:"<<map_index <<"patch cloud num:"<<vertexs.size()<<std::endl;
        outFile<<"After R search ,cloud frame num:"<<vertex_ids.size()<<std::endl;
        outFile<<"id  "<<"  bagname  "<<"  timestamp"<<std::endl;
        for(const auto& vertex_id:vertex_ids){
            VertexData vertex = id2vertex_map[vertex_id];
            outFile<<vertex_id<<","<<vertex.Bagname<<","<<vertex.Timestamp<<std::endl;
        }
        outFile.close();
        count++;
        if(count<=100)
            std::cout<<map_index<<" "; 
        if(count<=100&&count%10==0)std::cout<<std::endl;
        if(count>100&&count%10==0) std::cout<<". ";
    }
    std::cout<<std::endl;
    std::cout<<"Finish to save patch info!========>"<<std::endl;
    return;
}

void Adjustmap(std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>>& spatial_map,std::string file_path)
{
    std::vector<XYZID<>> neighbor_indexs;//保存合并的键
    std::vector<XYZID<>> keys_to_erase; // 保存要删除的键
    std::unordered_map<XYZID<>, std::vector<std::string>, XYZID<>>sub_spatial_map;// 保存要分裂的键
    for (auto it = spatial_map.begin(); it != spatial_map.end();++it ) 
    {
        const XYZID<>& key = it->first;
        std::vector<VertexData>& value = it->second;
        if(value.size()>=50||value.size()==0) {continue;}
        else if(value.size()<50)
            DFS(spatial_map,key,file_path,value,neighbor_indexs,keys_to_erase);
    }
    // 在循环外部删除键
    for (const auto& key : keys_to_erase) {
        spatial_map.erase(key);
    }
}

void DFS(std::unordered_map<XYZID<>, std::vector<VertexData>, XYZID<>>& spatial_map,const XYZID<>&key,std::string file_path,
        std::vector<VertexData>&value,std::vector<XYZID<>>& neighbor_indexs,std::vector<XYZID<>>& keys_to_erase)
 {
    if(value.size()>=50||value.size()==0) return;
    int idx=key.xid,idy=key.yid,idz=key.zid,ids=key.sid;
    std::vector<int> nez={1,-1, 2,-2};
    for(int n=0;n<nez.size()-1&&value.size()>0;++n)
    {
        XYZID<> nei_index = {static_cast<int64_t>(idx), static_cast<int64_t>(idy), static_cast<int64_t>(idz+nez[n])};
        auto neighbor_it = spatial_map.find(nei_index);
        if (neighbor_it != spatial_map.end())
        {
            std::vector<VertexData>& neighbor_value=spatial_map.find({nei_index})->second;//spatial_map[nei_index];
            if(neighbor_value.size()>0)
            {
                value.insert(value.end(),neighbor_value.begin(),neighbor_value.end());
                keys_to_erase.push_back(nei_index);
                neighbor_indexs.push_back(nei_index);
                neighbor_value.clear();
                if(value.size()>=50)
                {
                   
                    std::string merge_indece=file_path+"/"+std::to_string(idx)+"_"+std::to_string(idy)+"_"+std::to_string(idz)+".txt";
                    std::ofstream outfile(merge_indece, std::ios_base::app);
                    if (outfile.is_open()) {
                        for(int i=0;i<neighbor_indexs.size();++i)
                        {
                            outfile << neighbor_indexs[i].xid << " " << neighbor_indexs[i].yid << " " << neighbor_indexs[i].zid << std::endl; 
                        }
                        outfile.close();
                        // std::cout << "File created: " << merge_indece << std::endl;
                        // LogInfo("Merge index's file created: %s!\n",merge_indece.c_str());
                    } else {
                        // std::cerr << "Unable to open file: " << merge_indece << std::endl;
                        LogError("Unable to open merge index's file: %s!\n",merge_indece.c_str());
                    }
                    neighbor_indexs.clear();
                    return;
                }
            }
        }
    }
    std::vector<int> nei = {1, 0, -1, 0, 1};//, 1, -1, -1, 1};
    // bool found_neighbor = false;
    for(int n=0;n<nei.size()-1;++n)
    {
        int ex=nei[n],ey=nei[n+1];
        XYZID<> nei_index = {static_cast<int64_t>(idx+ex), static_cast<int64_t>(idy+ey), static_cast<int64_t>(idz)};
        auto neighbor_it = spatial_map.find(nei_index);
        if (neighbor_it != spatial_map.end())
        {
            std::vector<VertexData>& neighbor_value=spatial_map.find({nei_index})->second;//spatial_map[nei_index];
            if(neighbor_value.size()>0)
            {
                neighbor_value.insert(neighbor_value.end(),value.begin(),value.end());
                // found_neighbor = true;
                neighbor_indexs.push_back(key);
                keys_to_erase.push_back(key); // 添加要删除的键     
                value.clear();
                if(neighbor_value.size()>=50)
                {
                    std::string merge_indece=file_path+"/"+std::to_string(idx+ex)+"_"+std::to_string(idy+ey)+"_"+std::to_string(idz)+".txt";
                    std::ofstream outfile(merge_indece, std::ios_base::app);
                    if (outfile.is_open()) {
                        for(int i=0;i<neighbor_indexs.size();++i)
                        {
                            outfile << neighbor_indexs[i].xid << " " << neighbor_indexs[i].yid << " " << neighbor_indexs[i].zid << std::endl; 
                        }
                        outfile.close();
                        // LogInfo("Merge index's file created: %s!\n",merge_indece.c_str());
                    } else {
                        LogError("Unable to open merge index's file: %s!\n",merge_indece.c_str());
                        // std::cerr << "Unable to open file: " << merge_indece << std::endl;
                    }
                    neighbor_indexs.clear();
                    return;
                }
                else 
                {
                    // neighbor_indexs.push_back(key);
                    DFS(spatial_map,nei_index,file_path,neighbor_value,neighbor_indexs,keys_to_erase);
                    return;    
                }                
            }
            else{
                if(n==nei.size()-1)
                {
                    std::string merge_indece=file_path+"/"+std::to_string(idx+ex)+"_"+std::to_string(idy+ey)+"_"+std::to_string(idz)+".txt";
                    std::ofstream outfile(merge_indece, std::ios_base::app);
                    if (outfile.is_open()) {
                        for(int i=0;i<neighbor_indexs.size();++i)
                        {
                            outfile << neighbor_indexs[i].xid << " " << neighbor_indexs[i].yid << " " << neighbor_indexs[i].zid << std::endl; 
                        }
                        outfile.close();
                        // LogInfo("Merge index's file created: %s!\n",merge_indece.c_str());
                    } else {
                        LogError("Unable to open merge index's file: %s!\n",merge_indece.c_str());
                        // std::cerr << "Unable to open file: " << merge_indece << std::endl;
                    }
                    neighbor_indexs.clear();
                    return;
                }
                else continue;
            }
                   
        }
                 
    }
 }

