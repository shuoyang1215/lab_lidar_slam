#include <ros/ros.h>
#include "objectcluster_obj/objseg_grid_obj.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "objseg_gridclass");

    ros::NodeHandle nh("~");
//    ros::NodeHandle nh;

    ObjSegGrid_obj SegMethod(nh);

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
//        cout<<fixed<<"while: "<<ros::Time::now().toSec()<<endl;
//        std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
//        cout<<"while: "<<t0<<endl;

        SegMethod.ObjSegGridRun();
//        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        ros::spinOnce();
//        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
//        std::chrono::duration<double, std::milli> d01 = t1 - t0;
//        std::chrono::duration<double, std::milli> d12 = t2 - t1;
//        std::chrono::duration<double, std::milli> dall = t2 - t0;
        // if (d01.count()>5 || d12.count()>5 || dall.count()>5){
        //     std::cout << "main01:" << d01.count() << " main12:" << d12.count()
        //             << " mainall:" << dall.count() << endl;
        // }
        rate.sleep();
    }
    return 0;
}


