/************************************************************************/
/*                                                                      */
/*                                  公共处理类                          */
/*                                                                      */
/*                                                                      */
/************************************************************************/
    
#ifndef __COMMON__
#define __COMMON__
#include <iostream>
#include <sstream>
#include <vector>
#include <mutex>
#include <thread>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/time.h>
#include <sys/prctl.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <cstring>
#include <cstdio>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <time.h>
#include <pwd.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>


namespace xsproto {
namespace common {

#define SetToNULL(p) p = NULL

/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                 日志输出打印                         */
/*                                                                      */
/*                                                                      */
/************************************************************************/

//获取当前系统时间
extern bool getCurrentTime(char* t);

//获取当前系统时间对应的毫秒
extern double getCurrentMillSecond();


class TraceLog
{
public:
    TraceLog();
    ~TraceLog();
    /**
     * @brief setTraceLog: 该函数用于初始化LOG
     * @param string program: 程序名
     * @param log_path: LOG文件夹路径，由自己指定或者获取环境变量
     * @param int level: LOG等级，仅用于调试模式
     * @param int fflush_mode: 刷新模式————1：实时刷新，其他：异步单度刷新
     * @param int reserved_day: 日志保存时间，单位天
     * @param int divide_log: 日志分类模式————0:一个文件，1:LOG分为INFO ERROR DEBUG三个文件
     *  @param long maxline: log文件保存的最大行数
     * @return 成功标志位
     */
    bool setTraceLog(std::string program,
                     std::string log_path,
                     int level=0,
                     int fflush_mode=0,
                     int reserved_day=2,
                     int divide_log=0,
                     long maxline=100000,
                     int save_interval=1000000);
    FILE* getTraceLog(int log_mode=0);
    void ResetNewLogFile();
    int getDivideLog();
    int getTraceLevel();
    int getTraceFlush();
    bool m_main_thread_state;
    std::vector<std::string> m_log_list;
    std::vector<std::string> m_log_info_list;
    std::vector<std::string> m_log_error_list;
    std::vector<std::string> m_log_debug_list;
    std::mutex m_mtx_log;

private:
    static void *SaveLogCallback(void *arg);
    void CloseAllFiles();
    int DayDiff(int year_start, int month_start, int day_start, int year_end, int month_end, int day_end);
    int RemoveFileList(const char *basePath, int reserved_day=2);
    int LoopMakeLogDir(std::string log_folder);
    FILE *m_logfp;
    FILE *m_log_info_fp;
    FILE *m_log_error_fp;
    FILE *m_log_debug_fp;

    long m_log_lines;
    long m_log_info_lines;
    long m_log_error_lines;
    long m_log_debug_lines;

    std::string m_logname;

    int m_divide_log;
    int m_level;
    int m_fflush_mode;
    long m_maxline;
    int m_save_interval;
    std::thread m_savelog_thread;

};


typedef boost::serialization::singleton<xsproto::common::TraceLog> GlobeLogFp; //ubuntu
#define SINGLETON_LOG xsproto::common::GlobeLogFp::get_mutable_instance()      //ubuntu


//这里使用了logfp是文件句柄指针
#define LogInfo(form, info...) do\
{\
    char t[32] = {0};\
    xsproto::common::getCurrentTime(t);\
    if(SINGLETON_LOG.getDivideLog() == 1)\
    {\
        if (SINGLETON_LOG.getTraceLog(1) != NULL)\
        {\
            if (SINGLETON_LOG.getTraceFlush() == 1)\
            {\
                fprintf(SINGLETON_LOG.getTraceLog(1), "[%s][INFO]" form, t, ##info);\
                fflush(SINGLETON_LOG.getTraceLog(1));\
            }\
            else\
            {\
                char log_data[2048];\
                sprintf(log_data, "[%s][INFO]" form, t, ##info);\
                std::string log_data_str = log_data;\
                SINGLETON_LOG.m_mtx_log.lock();\
                SINGLETON_LOG.m_log_info_list.push_back(log_data_str);\
                SINGLETON_LOG.m_mtx_log.unlock();\
            }\
        }\
        else\
        {\
            printf("[%s][INFO]" form, t, ##info);\
        }\
    }\
    else\
    {\
        if (SINGLETON_LOG.getTraceLog(0) != NULL)\
        {\
            if (SINGLETON_LOG.getTraceFlush() == 1)\
            {\
                fprintf(SINGLETON_LOG.getTraceLog(0), "[%s][INFO]" form, t, ##info);\
                fflush(SINGLETON_LOG.getTraceLog(0));\
            }\
            else\
            {\
                char log_data[2048];\
                sprintf(log_data, "[%s][INFO]" form, t, ##info);\
                std::string log_data_str = log_data;\
                SINGLETON_LOG.m_mtx_log.lock();\
                SINGLETON_LOG.m_log_list.push_back(log_data_str);\
                SINGLETON_LOG.m_mtx_log.unlock();\
            }\
        }\
        else\
        {\
            printf("[%s][INFO]" form, t, ##info);\
        }\
    }\
}while(0)
    

#define LogError(form, info...) do\
{\
    char t[32] = {0};\
    xsproto::common::getCurrentTime(t);\
    if(SINGLETON_LOG.getDivideLog() == 1)\
    {\
        if (SINGLETON_LOG.getTraceLog(2) != NULL)\
        {\
            if (SINGLETON_LOG.getTraceFlush() == 1)\
            {\
                fprintf(SINGLETON_LOG.getTraceLog(2), "[%s][ERROR]" form, t, ##info);\
                fflush(SINGLETON_LOG.getTraceLog(2));\
            }\
            else\
            {\
                char log_data[2048];\
                sprintf(log_data, "[%s][ERROR]" form, t, ##info);\
                std::string log_data_str = log_data;\
                SINGLETON_LOG.m_mtx_log.lock();\
                SINGLETON_LOG.m_log_error_list.push_back(log_data_str);\
                SINGLETON_LOG.m_mtx_log.unlock();\
            }\
        }\
        else\
        {\
            printf("[%s][ERROR]" form, t, ##info);\
        }\
    }\
    else\
    {\
        if (SINGLETON_LOG.getTraceLog(0) != NULL)\
        {\
            if (SINGLETON_LOG.getTraceFlush() == 1)\
            {\
                fprintf(SINGLETON_LOG.getTraceLog(0), "[%s][ERROR]" form, t, ##info);\
                fflush(SINGLETON_LOG.getTraceLog(0));\
            }\
            else\
            {\
                char log_data[2048];\
                sprintf(log_data, "[%s][ERROR]" form, t, ##info);\
                std::string log_data_str = log_data;\
                SINGLETON_LOG.m_mtx_log.lock();\
                SINGLETON_LOG.m_log_list.push_back(log_data_str);\
                SINGLETON_LOG.m_mtx_log.unlock();\
            }\
        }\
        else\
        {\
            printf("[%s][ERROR]" form, t, ##info);\
        }\
    }\
}while(0)


#define LogDebug(level, form, info...) do\
{\
    if (SINGLETON_LOG.getTraceLevel() >= level)\
    {\
        char t[32] = {0};\
        xsproto::common::getCurrentTime(t);\
        if(SINGLETON_LOG.getDivideLog() == 1)\
        {\
            if (SINGLETON_LOG.getTraceLog(3) != NULL)\
            {\
                if (SINGLETON_LOG.getTraceFlush() == 1)\
                {\
                    fprintf(SINGLETON_LOG.getTraceLog(3), "[%s][DEBUG]" form, t, ##info);\
                    fflush(SINGLETON_LOG.getTraceLog(3));\
                }\
                else\
                {\
                    char log_data[2048];\
                    sprintf(log_data, "[%s][DEBUG]" form, t, ##info);\
                    std::string log_data_str = log_data;\
                    SINGLETON_LOG.m_mtx_log.lock();\
                    SINGLETON_LOG.m_log_debug_list.push_back(log_data_str);\
                    SINGLETON_LOG.m_mtx_log.unlock();\
                }\
            }\
            else\
            {\
                printf("[%s][DEBUG]" form, t, ##info);\
            }\
        }\
        else\
        {\
            if (SINGLETON_LOG.getTraceLog(0) != NULL)\
            {\
                if (SINGLETON_LOG.getTraceFlush() == 1)\
                {\
                    fprintf(SINGLETON_LOG.getTraceLog(0), "[%s][DEBUG]" form, t, ##info);\
                    fflush(SINGLETON_LOG.getTraceLog(0));\
                }\
                else\
                {\
                    char log_data[2048];\
                    sprintf(log_data, "[%s][DEBUG]" form, t, ##info);\
                    std::string log_data_str = log_data;\
                    SINGLETON_LOG.m_mtx_log.lock();\
                    SINGLETON_LOG.m_log_list.push_back(log_data_str);\
                    SINGLETON_LOG.m_mtx_log.unlock();\
                }\
            }\
            else\
            {\
                printf("[%s][DEBUG]" form, t, ##info);\
            }\
        }\
    }\
}while(0)


/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                          运行时间计时                                */
/*                                                                      */
/*                                                                      */
/************************************************************************/
class timer
{
public:
    timer();
    void begin();
    double elapsed(); //返回的是微妙
    int calElapsedMin();  //返回使用的分钟
    int calElapsedSec();  //返回使用的秒
    int calElapsedMill();    //返回的是毫秒
    void getRecordTime(struct timeval& _t);

private:
    struct timeval _start_time;
};


/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                    程序信号操作                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
extern bool setExitCode();
extern bool getExitCode();
extern bool checkExitCode();


/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                       socket     操作                                */
/*                                                                      */
/*                                                                      */
/************************************************************************/

#define PORT 2020    //通讯端口
#define BACKLOG 20
#define MAXFILEPATH 256

#define STRCMPFALSERETURN(a, b) do\
{\
    if (strcmp(a, b) != 0)\
    {\
        return false;\
    }\
}while(0)

#define STRCMPFALSECONTINUE(a, b) do\
{\
    if (strcmp(a, b) != 0)\
    {\
        sleep(3);\
        continue;\
    }\
}while(0)


/*
 自定义应用层协议
 第一个字节表示命令类型，
 接下来4个字节表示数据长度，
 再接下来就是具体数据
*/
enum LENGTH
{
    header_length = 1,
    data_length = 4,
    max_data_length = 1000
};

class message
{
public:
    message(int fd);

    void reset(int fd);

    bool recvMsg();

    bool sendMsg(const char* mode, const char* data);

    const char* mode() const;

    int length() const;

    const char* data() const;

private:
    void clear();

private:
    char m_mode[header_length + 1];
    int  m_length;
    char m_data[max_data_length + 1];
    int  m_fd;
};


//信号量
union semun
{
    int  val;
    struct semid_ds* buf;
    unsigned short* array;
};


class sem
{
public:
    //create
    bool init(int Key, int val = 1);
    bool create(int key);
    bool get(int key);
    //lock
    void lock();
    //unlock
    void unlock();
    bool del();
    void wait();
    void post();
private:
    int m_semid;
    //FILE *logfp;
};


/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                             file操作                                 */
/*                                                                      */
/*                                                                      */
/************************************************************************/

class file
{
public:
    file();
    ~file();
    //创建文件
    bool Create(const char* str, char* filename);
    //打开文件
    bool Open(const char* filename);
    //读取文件
    int ReadData(char* data);
    //写入文件
    bool WriteData(const char* data);
    //检查文件
    size_t FileSize();
private:
    char _fileName[MAXFILEPATH];
    size_t _fileSize;
    int  _fileDes;
};


/************************************************************************/
/*                                                                      */
/*                           读取ini配置文件                            */
/*                                                                      */
/*                                                                      */
/************************************************************************/
class iniConf
{
public:
    iniConf();
    iniConf(const char* fileConf);
    ~iniConf();
    bool openFile(const char* fileConf);
    int getValue(std::string Key, int Default, char Sep = '.');
    std::string getValue(std::string Key, std::string Default, char Sep = '.');
private:
    boost::property_tree::ptree m_pt;
};


typedef boost::serialization::singleton<xsproto::common::iniConf> GlobeIniConf; //ubuntu
#define SINGLETON_INICONF GlobeIniConf::get_mutable_instance()      //ubuntu

}  // namespace common
}  // namespace xsproto
#endif
