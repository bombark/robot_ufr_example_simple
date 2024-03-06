#include <stdio.h>
#include <stdlib.h>
#include <ufr.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#define LIDAR_SIZE 360

using namespace std;
using namespace cv;



int main() {
    link_t lidar_dev = ufr_sys_open("lidar", "@new zmq:topic @host 192.168.43.141 @port 5002 @coder msgpack:obj");
    ufr_start_subscriber(&lidar_dev, NULL);

    link_t pose_dev = ufr_sys_open("pose", "@new zmq:topic @host 192.168.43.141 @port 5004 @coder msgpack:obj");
    ufr_start_subscriber(&pose_dev, NULL);

    Mat map(600, 600, CV_8UC1);

    int world_r_y = 300;
    int world_r_x = 300;

    while (1) {
        map.setTo(0);
        float lidar[LIDAR_SIZE];
        ufr_get(&lidar_dev, "^af", LIDAR_SIZE, lidar);
        
        float robot_x, robot_y, robot_th;
        ufr_get(&pose_dev, "^fff", &robot_x, &robot_y, &robot_th);
        float robot_th_rad = robot_th * M_PI / 180.0;

        printf("%f %f %f\n", robot_x, robot_y, robot_th );

        float angulo = -M_PI/2.0;
        float diff_angulo = 3.141592*2 / LIDAR_SIZE;
        for (int i=0;   i<LIDAR_SIZE;   i++, angulo += diff_angulo) {
            float dist = lidar[i] / 50.0;
            // printf("%f ", dist);
            uint16_t y = world_r_y - ( sin(angulo-robot_th_rad) * dist * 1.0 );
            uint16_t x = world_r_x + ( cos(angulo-robot_th_rad) * dist * 1.0 );
            
            map.at<uint8_t>(y,x) = 255;
        }
        // printf("\n");
        
        imshow("map", map);
        waitKey(10);
    }
    return 0;
}