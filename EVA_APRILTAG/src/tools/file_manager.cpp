
#include "tools/file_manager.hpp"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

#ifdef WIN32
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#endif
#define MAX_PATH_LEN 256
#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

namespace eva_april {
    bool FileManager::CreateFile(std::ofstream &ofs, std::string file_path) {
        ofs.close();
        boost::filesystem::remove(file_path.c_str());

        ofs.open(file_path.c_str(), std::ios::out);
        if (!ofs) {
            return false;
        }

        return true;
    }

    bool FileManager::InitDirectory(std::string directory_path, std::string use_for) {
        if (boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::remove_all(directory_path);
        }

        return CreateDirectory(directory_path, use_for);
    }

    int32_t FileManager::createDirectory(const std::string &directoryPath){
        uint32_t dirPathLen = directoryPath.length();
        if (dirPathLen > MAX_PATH_LEN){
            std::cout << "not able to create " << std::endl;
            return 1;
        }

        char tmpDirPath[MAX_PATH_LEN] = {0};
        for (uint32_t i = 0; i < dirPathLen; ++i){
            tmpDirPath[i] = directoryPath[i];
            if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/'){
                if (ACCESS(tmpDirPath, 0) != 0){
                    int32_t ret = MKDIR(tmpDirPath);
                    if (ret != 0){
                        return ret;
                    }
                }
            }
        }
        return 0;
    }

    bool FileManager::CreateDirectory(std::string directory_path, std::string use_for) {
        if (!boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::create_directory(directory_path);
        }
        if (!boost::filesystem::is_directory(directory_path)) {
            return false;
        }
        std::cerr << use_for << "存放地址：" << std::endl << directory_path << std::endl << std::endl;
        return true;
    }

    bool FileManager::CreateDirectory(std::string directory_path) {
        if (!boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::create_directory(directory_path);
        }
        if (!boost::filesystem::is_directory(directory_path)) {
            return false;
        }

        return true;
    }

    void FileManager::replace_str(std::string &str, const std::string &before, const std::string &after) {
        for (std::string::size_type pos(0); pos != std::string::npos; pos += after.length()) {
            pos = str.find(before, pos);
            if (pos != std::string::npos)
                str.replace(pos, before.length(), after);
            else
                break;
        }
    }
}