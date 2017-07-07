//
// Created by osboxes on 26/06/17.
//

#include <iostream>
#include "VoxelsConversion.h"

int main(){
    float x = 10;
    float y = 10;
    float z = 10;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->height=1;
    int cont=0;
    for(float i=-x/2; i<x/2; i+=0.25){
        for(float j=-y/2; j<y/2; j+=0.25){
            for(float k=-z/2; k<z/2; k+=0.25){
                cont++;
                cloud->width=cont;
                cloud->points.resize(cont);
                cloud->points[cont-1].x=25+i;
                cloud->points[cont-1].y=25+j;
                cloud->points[cont-1].z=25+k;
            }
        }
    }
}