
#ifndef GLOBAL_FUSION_TOOLS_FILE_MANAGER_HPP_
#define GLOBAL_FUSION_TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace eva_april {
    class FileManager {
    public:
        static bool CreateFile(std::ofstream &ofs, std::string file_path);

        static bool InitDirectory(std::string directory_path, std::string use_for);

        static int32_t createDirectory(const std::string &directoryPath);

        static bool CreateDirectory(std::string directory_path, std::string use_for);

        static bool CreateDirectory(std::string directory_path);

        static void replace_str(std::string &str, const std::string &before, const std::string &after);
    };
}

#endif
