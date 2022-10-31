//
// Created by csl on 10/31/22.
//

#include "hello_with_pcl.h"
#include "pcl-1.10/pcl/point_types.h"

namespace ns_hello_with_pcl {
    void HelloWorld() {
        pcl::PointXYZ p;
        p.x = 1.0f;
        p.y = 2.0f;
        p.z = 3.0f;
        std::cout << "this is a pcl point: " << p << std::endl;
    }
}
