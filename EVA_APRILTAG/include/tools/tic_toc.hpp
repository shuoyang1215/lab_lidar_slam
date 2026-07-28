

#ifndef GLOBAL_FUSION_TOOLS_TIC_TOC_HPP_
#define GLOBAL_FUSION_TOOLS_TIC_TOC_HPP_

#include <ctime>
#include <cstdlib>
#include <chrono>
#include "glog/logging.h"
#include "config.h"
namespace eva_april {
    class TicToc {
    public:
        TicToc(std::string head = "", bool screen = false) {
            tic();
            head_ = head;
            screen_ = screen;
            meanTime = 0.0;
            maxTime = 0.0;
            sumTime = 0.0;
            cnt = 0;
        }

        void tic() {
            start = std::chrono::system_clock::now();
        }

        double toc() {
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            curr_time = elapsed_seconds.count();
            cnt++;
            sumTime = sumTime + curr_time;
            meanTime = sumTime / cnt;
            if(maxTime < curr_time)
                maxTime = curr_time;
            print();
            return curr_time;
        }

        double get_cur_time(){
            return curr_time;
        }

        void print() {
            if(!screen_){
                LOG_IF(INFO, Config::use_glog) << head_ << " " << "cur: " << curr_time << " " << "mean: " << meanTime << " " << "max: " << maxTime <<std::endl;
            }
            else{
                LOG_IF(INFO, Config::use_glog) << head_ << " " << "cur: " << curr_time << " " << "mean: " << meanTime << " " << "max: " << maxTime <<std::endl;
                std::cerr << head_ << " " << "cur: " << curr_time << " " << "mean: " << meanTime << " " << "max: " << maxTime <<std::endl;
            }
            return;
        }

    private:
        std::chrono::time_point<std::chrono::system_clock> start, end;
        double meanTime, maxTime, sumTime, curr_time;
        uint64_t cnt;
        bool screen_;
        std::string head_;
    };
}
#endif
