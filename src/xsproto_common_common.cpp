#include "xsproto_common_common.h"

using namespace std;

namespace xsproto
{
    namespace common
    {

        bool getCurrentTime(char *t)
        {
            memset(t, 0, strlen(t));
            struct timeval _tv;
            struct tm *_now;
            time_t _time;

            gettimeofday(&_tv, NULL);
            _time = _tv.tv_sec;
            _now = localtime(&_time);

            sprintf(t, "%4d-%02d-%02d %02d:%02d:%02d.%06ld",
                    1900 + _now->tm_year,
                    1 + _now->tm_mon,
                    _now->tm_mday,
                    _now->tm_hour,
                    _now->tm_min,
                    _now->tm_sec,
                    _tv.tv_usec);
            return true;
        }

        double getCurrentMillSecond()
        {
            struct timeval _tv;
            gettimeofday(&_tv, NULL);
            return _tv.tv_sec * 1000 + _tv.tv_usec / 1000;
        }

        TraceLog::TraceLog()
        {
            m_logfp = NULL;
            m_log_info_fp = NULL;
            m_log_error_fp = NULL;
            m_log_debug_fp = NULL;
            m_log_lines = 0;
            m_log_info_lines = 0;
            m_log_error_lines = 0;
            m_log_debug_lines = 0;

            m_level = 0;
            m_fflush_mode = 0;
            m_save_interval = 1000000;
            m_main_thread_state = true;
        }

        TraceLog::~TraceLog()
        {
            m_main_thread_state = false;

            if (m_savelog_thread.joinable())
            {
                m_savelog_thread.join();
            }

            if (m_divide_log == 1)
            {
                if (m_log_info_list.size() > 0)
                {
                    for (unsigned int i = 0; i < m_log_info_list.size(); ++i)
                    {
                        fprintf(getTraceLog(1), "%s", m_log_info_list[i].c_str());
                    }
                    fflush(getTraceLog(1));
                }

                if (m_log_error_list.size() > 0)
                {
                    for (unsigned int i = 0; i < m_log_error_list.size(); ++i)
                    {
                        fprintf(getTraceLog(2), "%s", m_log_error_list[i].c_str());
                    }
                    fflush(getTraceLog(2));
                }

                if (m_log_debug_list.size() > 0)
                {
                    for (unsigned int i = 0; i < m_log_debug_list.size(); ++i)
                    {
                        fprintf(getTraceLog(3), "%s", m_log_debug_list[i].c_str());
                    }
                    fflush(getTraceLog(3));
                }
            }
            else
            {
                if (m_log_list.size() > 0)
                {
                    for (unsigned int i = 0; i < m_log_list.size(); ++i)
                    {
                        fprintf(getTraceLog(0), "%s", m_log_list[i].c_str());
                    }
                    fflush(getTraceLog(0));
                }
            }

            CloseAllFiles();
        }

        void TraceLog::CloseAllFiles()
        {
            if (m_logfp != NULL)
            {
                fclose(m_logfp);
            }
            if (m_log_info_fp != NULL)
            {
                fclose(m_log_info_fp);
            }
            if (m_log_error_fp != NULL)
            {
                fclose(m_log_error_fp);
            }
            if (m_log_debug_fp != NULL)
            {
                fclose(m_log_debug_fp);
            }
        }

        int TraceLog::DayDiff(int year_start, int month_start, int day_start, int year_end, int month_end, int day_end)
        {
            int y2, m2, d2;
            int y1, m1, d1;

            m1 = (month_start + 9) % 12;
            y1 = year_start - m1 / 10;
            d1 = 365 * y1 + y1 / 4 - y1 / 100 + y1 / 400 + (m1 * 306 + 5) / 10 + (day_start - 1);

            m2 = (month_end + 9) % 12;
            y2 = year_end - m2 / 10;
            d2 = 365 * y2 + y2 / 4 - y2 / 100 + y2 / 400 + (m2 * 306 + 5) / 10 + (day_end - 1);

            return abs(d2 - d1);
        }

        int TraceLog::RemoveFileList(const char *basePath, int reserved_day)
        {
            DIR *dir;
            struct dirent *ptr;
            char path[1000];

            if ((dir = opendir(basePath)) == NULL)
            {
                return 0;
            }

            while ((ptr = readdir(dir)) != NULL)
            {
                if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) /// current dir OR parrent dir
                    continue;
                else if (ptr->d_type == 8) /// file
                {
                    sprintf(path, "%s%s", basePath, ptr->d_name);
                    timeval tv;
                    gettimeofday(&tv, NULL);
                    tm *curTime = localtime(&tv.tv_sec);
                    string str = ptr->d_name;
                    int index = str.find(".log");
                    if (index >= 19)
                    {
                        string str2 = str.substr(index - 19, index);
                        int year = 0, month = 0, day = 0;
                        sscanf(str2.c_str(), "%d-%d-%d", &year, &month, &day);
                        if (DayDiff(year, month, day, curTime->tm_year + 1900,
                                    curTime->tm_mon + 1, curTime->tm_mday) > reserved_day)
                        {
                            printf("delete useless log file: %s\n", path);
                            remove(path);
                        }
                    }
                    else
                    {
                        printf("delete useless log file: %s\n", path);
                        remove(path);
                    }
                }
            }
            closedir(dir);
            return 1;
        }

        int TraceLog::LoopMakeLogDir(std::string log_folder)
        {
            if (!access(log_folder.c_str(), F_OK)) /* 判断目标文件夹是否存在 */
            {
                return 1;
            }
            char path[512];      /* 目标文件夹路径 */
            char *path_buf;      /* 目标文件夹路径指针 */
            char temp_path[512]; /* 存放临时文件夹路径 */
            char *temp;          /* 单级文件夹名称 */
            int temp_len;        /* 单级文件夹名称长度 */
            memset(path, 0, sizeof(path));
            memset(temp_path, 0, sizeof(temp_path));
            strcat(path, log_folder.c_str());
            path_buf = path;
            while ((temp = strsep(&path_buf, "/")) != NULL) /* 拆分路径 */
            {
                temp_len = strlen(temp);
                if (0 == temp_len)
                {
                    continue;
                }
                strcat(temp_path, "/");
                strcat(temp_path, temp);
                if (-1 == access(temp_path, F_OK)) /* 不存在则创建 */
                {
                    if (-1 == mkdir(temp_path, 0744))
                    {
                        return 0;
                    }
                }
            }
            return 1;
        }

        bool TraceLog::setTraceLog(std::string program, std::string log_path, int level, int fflush_mode,
                                   int reserved_day, int divide_log, long maxline, int save_interval)
        {
            m_divide_log = divide_log;
            m_level = level;
            m_fflush_mode = fflush_mode;
            m_maxline = maxline;
            m_save_interval = save_interval;

            std::string logname;
            std::string::size_type folder_pos = program.find_last_of("/");
            if (folder_pos != std::string::npos)
            {
                program = program.substr(folder_pos + 1, program.size() - folder_pos - 1);
            }

            if (program.size() != 0)
            {
                if (log_path.size() != 0)
                {
                    // log路径增加
                    if (log_path.back() != '/')
                    {
                        log_path.append("/");
                    }
                    logname = log_path + program + "/";
                }
                else
                {
                    logname = "/data/log/" + program + "/";
                }
                // 创建新日志文件夹
                LoopMakeLogDir(logname);

                // 删除超限日志
                RemoveFileList(logname.c_str(), reserved_day);

                // 新日志名称
                logname = logname + program + "_";
                m_logname = logname;
                ResetNewLogFile();

                if (m_fflush_mode != 1)
                {
                    m_savelog_thread = std::thread(SaveLogCallback, this);
                }
            }
            else
            {
                return false;
            }

            return true;
        }

        void TraceLog::ResetNewLogFile()
        {
            // 获取最新文件名
            std::string logname = m_logname;
            char t[32] = {0};
            xsproto::common::getCurrentTime(t);
            char d[32] = {0};
            memcpy(d, t, 19);
            d[10] = '-';
            d[13] = '-';
            d[16] = '-';
            logname = logname + std::string(d);
            // 如果有打开的文件则关闭文件
            CloseAllFiles();
            // 重新打开新文件
            if (m_divide_log == 1)
            {
                std::string info_logname = logname + ".log.info";
                m_log_info_fp = fopen(info_logname.c_str(), "a");
                std::string error_logname = logname + ".log.error";
                m_log_error_fp = fopen(error_logname.c_str(), "a");
                std::string debug_logname = logname + ".log.debug";
                m_log_debug_fp = fopen(debug_logname.c_str(), "a");
            }
            else
            {
                logname = logname + ".log";
                m_logfp = fopen(logname.c_str(), "a");
            }
        }

        void *TraceLog::SaveLogCallback(void *arg)
        {
            TraceLog *th = (TraceLog *)arg;
            while (th->m_main_thread_state)
            {
                if (th->m_divide_log == 1)
                {
                    th->m_mtx_log.lock();
                    std::vector<std::string> log_info_list = th->m_log_info_list;
                    std::vector<std::string> log_error_list = th->m_log_error_list;
                    std::vector<std::string> log_debug_list = th->m_log_debug_list;
                    th->m_log_info_list.clear();
                    th->m_log_error_list.clear();
                    th->m_log_debug_list.clear();
                    th->m_mtx_log.unlock();

                    if (log_info_list.size() > 0)
                    {
                        for (unsigned int i = 0; i < log_info_list.size(); ++i)
                        {
                            fprintf(th->getTraceLog(1), "%s", log_info_list[i].c_str());
                        }
                        fflush(th->getTraceLog(1));
                    }

                    if (log_error_list.size() > 0)
                    {
                        for (unsigned int i = 0; i < log_error_list.size(); ++i)
                        {
                            fprintf(th->getTraceLog(2), "%s", log_error_list[i].c_str());
                        }
                        fflush(th->getTraceLog(2));
                    }

                    if (log_debug_list.size() > 0)
                    {
                        for (unsigned int i = 0; i < log_debug_list.size(); ++i)
                        {
                            fprintf(th->getTraceLog(3), "%s", log_debug_list[i].c_str());
                        }
                        fflush(th->getTraceLog(3));
                    }

                    th->m_log_info_lines = th->m_log_info_lines + log_info_list.size();
                    th->m_log_error_lines = th->m_log_error_lines + log_error_list.size();
                    th->m_log_debug_lines = th->m_log_debug_lines + log_debug_list.size();
                    if (th->m_log_info_lines > th->m_maxline)
                    {
                        th->m_log_info_lines = 0;
                        th->m_log_error_lines = 0;
                        th->m_log_debug_lines = 0;
                        th->ResetNewLogFile();
                    }
                    if (th->m_log_error_lines > th->m_maxline)
                    {
                        th->m_log_info_lines = 0;
                        th->m_log_error_lines = 0;
                        th->m_log_debug_lines = 0;
                        th->ResetNewLogFile();
                    }
                    if (th->m_log_debug_lines > th->m_maxline)
                    {
                        th->m_log_info_lines = 0;
                        th->m_log_error_lines = 0;
                        th->m_log_debug_lines = 0;
                        th->ResetNewLogFile();
                    }
                }
                else
                {
                    th->m_mtx_log.lock();
                    std::vector<std::string> log_list = th->m_log_list;
                    th->m_log_list.clear();
                    th->m_mtx_log.unlock();

                    if (log_list.size() > 0)
                    {
                        for (unsigned int i = 0; i < log_list.size(); ++i)
                        {
                            fprintf(th->getTraceLog(0), "%s", log_list[i].c_str());
                        }
                        fflush(th->getTraceLog(0));
                    }

                    th->m_log_lines = th->m_log_lines + log_list.size();
                    if (th->m_log_lines > th->m_maxline)
                    {
                        th->m_log_lines = 0;
                        th->ResetNewLogFile();
                    }
                }

                usleep(th->m_save_interval);
            }
            return nullptr;  // add by guan
        }

        FILE *TraceLog::getTraceLog(int log_mode)
        {
            if (log_mode == 1)
                return m_log_info_fp;
            else if (log_mode == 2)
                return m_log_error_fp;
            else if (log_mode == 3)
                return m_log_debug_fp;
            else
                return m_logfp;
        }

        int TraceLog::getDivideLog()
        {
            return m_divide_log;
        }

        int TraceLog::getTraceLevel()
        {
            return m_level;
        }

        int TraceLog::getTraceFlush()
        {
            return m_fflush_mode;
        }

        /************************************************************************/
        /*                                                                      */
        /*                                                                      */
        /*                          运行时间计时                                */
        /*                                                                      */
        /*                                                                      */
        /************************************************************************/

        timer::timer()
        {
            gettimeofday(&_start_time, NULL);
        }

        void timer::begin()
        {
            gettimeofday(&_start_time, NULL);
        }

        double timer::elapsed()
        {
            struct timeval _end_time;
            gettimeofday(&_end_time, NULL);
            double timeuse = 1000000 * (_end_time.tv_sec - _start_time.tv_sec) + _end_time.tv_usec - _start_time.tv_usec;
            // timeuse /= 1000000;
            return timeuse;
        }

        int timer::calElapsedMin()
        {
            struct timeval _end_time;
            gettimeofday(&_end_time, NULL);
            return ((_end_time.tv_sec - _start_time.tv_sec) + (_end_time.tv_usec - _start_time.tv_usec) / 1000000) / 60;
        }

        int timer::calElapsedSec()
        {
            struct timeval _end_time;
            gettimeofday(&_end_time, NULL);
            return _end_time.tv_sec - _start_time.tv_sec + (_end_time.tv_usec - _start_time.tv_usec) / 1000000;
        }

        int timer::calElapsedMill()
        {
            struct timeval _end_time;
            gettimeofday(&_end_time, NULL);
            return 1000 * (_end_time.tv_sec - _start_time.tv_sec) + (_end_time.tv_usec - _start_time.tv_usec) / 1000;
        }

        void timer::getRecordTime(struct timeval &_t)
        {
            _t.tv_sec = _start_time.tv_sec;
            _t.tv_usec = _start_time.tv_usec;
        }

        /************************************************************************/
        /*                                                                      */
        /*                                                                      */
        /*                    程序信号操作                                      */
        /*                                                                      */
        /*                                                                      */
        /************************************************************************/

        bool setExitCode()
        {
            sigset_t newmask;
            sigset_t oldmask;

            //清除所有信号的阻塞标志
            sigemptyset(&newmask);
            sigaddset(&newmask, SIGINT);
            sigaddset(&newmask, SIGTERM);

            //更改进程的信号屏蔽字
            if (sigprocmask(SIG_BLOCK, &newmask, &oldmask) < 0)
            {
                return false;
            }
            return true;
        }

        bool getExitCode()
        {
            sigset_t pendmask;
            //取阻塞信号
            if (sigpending(&pendmask) < 0)
            {
                return false;
            }

            //判断某个信号是否被阻塞
            if (sigismember(&pendmask, SIGINT) || sigismember(&pendmask, SIGTERM))
            {
                return true;
            }
            return false;
        }

        /************************************************************************/
        /*                                                                      */
        /*                                                                      */
        /*                             file操作                                 */
        /*                                                                      */
        /*                                                                      */
        /************************************************************************/

        file::file()
        {
            memset(_fileName, 0, sizeof(_fileName));
            _fileSize = 0;
            _fileDes = 0;
        }

        file::~file()
        {
            close(_fileDes);
        }

        bool file::Create(const char *str, char *filename)
        {
            if (str[0] == 0)
            {
                return false;
            }
            std::vector<std::string> v;
            boost::split(v, str, boost::is_any_of(";"));
            if (v.size() != 3)
            {
                return false;
            }

            std::string path = v[0];
            strcpy(_fileName, v[1].c_str());
            _fileSize = atoi(v[2].c_str());
            std::string totallyName = path + "/" + v[1];
            strcpy(filename, totallyName.c_str());

            // boost::filesystem::path p(totallyName.c_str());
            //文件存在先删除
            // if (boost::filesystem::exists(p))
            //{
            //     boost::filesystem::remove(p);
            // }

            remove(totallyName.c_str());

            _fileDes = open(totallyName.c_str(), O_WRONLY | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP); // O_SYNC O_EXCL
            if (_fileDes == -1)
            {
                return false;
            }

            return true;
        }

        bool file::Open(const char *filename)
        {
            if (filename[0] == 0)
            {
                return false;
            }

            _fileDes = open(filename, O_RDONLY);
            if (_fileDes == -1)
            {
                return false;
            }
            return true;
        }

        bool file::WriteData(const char *data)
        {
            if (data[0] == 0)
            {
                return false;
            }
            int res = write(_fileDes, data, strlen(data));
            if (res == -1)
            {
                return false;
            }
            return true;
        }

        int file::ReadData(char *data)
        {
            memset(data, 0, strlen(data));
            // 0表示文件末尾
            //-1表示出错
            size_t read_bytes = read(_fileDes, data, max_data_length);
            if (read_bytes > 0 && read_bytes < strlen(data))
            {
                printf("read_bytes is %ld. strlen(data) is %ld\n", read_bytes, strlen(data));
                memset(data + read_bytes, 0, strlen(data) - read_bytes);
            }
            return read_bytes;
        }

        size_t file::FileSize()
        {
            return _fileSize;
        }

        /************************************************************************/
        /*                                                                      */
        /*                           读取ini配置文件                            */
        /*                                                                      */
        /*                                                                      */
        /************************************************************************/

        iniConf::iniConf()
        {
        }

        iniConf::iniConf(const char *fileConf)
        {
            openFile(fileConf);
        }

        iniConf::~iniConf()
        {
        }

        bool iniConf::openFile(const char *fileConf)
        {
            if (NULL == fileConf)
            {
                return false;
            }
            try
            {
                boost::property_tree::read_ini(fileConf, m_pt);
            }
            catch (exception &e)
            {
                cout << "iniConf::openFile -- parse file failed :" << e.what() << endl;
                return false;
            }

            return true;
        }

        int iniConf::getValue(std::string Key, int Default, char Sep)
        {
            try
            {
                return m_pt.get(boost::property_tree::ptree::path_type(Key, Sep), Default);
            }
            catch (exception &e)
            {
                cout << "iniConf::getvalue -- get value feiled:" << e.what() << endl;
                return Default;
            }
        }

        std::string iniConf::getValue(std::string Key, std::string Default, char Sep)
        {
            try
            {
                return m_pt.get(boost::property_tree::ptree::path_type(Key, Sep), Default);
            }
            catch (exception &e)
            {
                cout << "iniConf::getvalue -- get value feiled:" << e.what() << endl;
                return Default;
            }
        }

    } // namespace common
} // namespace xsproto
