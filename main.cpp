#include <iostream>
#include "VoxelsConversion.h"

int main() {

    BoxelsConversion bc;
    Clusters segmented_pcl;

    segmented_pcl = bc.RegionGrowingSegment("obj_0.pcd", 5.0, 1.5, 300);


    return 0;
}