/*
 * IRIS Localization and Mapping (LaMa) for ROS
 *
 * Copyright (c) 2022-today, Eurico Pedrosa, University of Aveiro - Portugal
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Aveiro nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

// ============================================================================
//  Header
// ============================================================================

#include <lama/graph_slam2d.h>
#include <ufr.h>

// ============================================================================
//  Main
// ============================================================================

int main(int argc, char *argv[]) {
    //
    double tmp = 0;
    Eigen::Vector2d pos(0,0);
    lama::Pose2D prior(pos, tmp);

    link_t scan_sub = ufr_subscriber("@new zmq:topic @host 127.0.0.1 @port 5001 @coder msgpack");
    link_t odom_sub = ufr_subscriber("@new zmq:topic @host 127.0.0.1 @port 5004 @coder msgpack");

    //
    lama::GraphSlam2D slam;
    slam.Init(prior);
    
    //
    while (1) {
        float p1,p2;
        ufr_get(&odom_sub, "^ff", &p1, &p2);

        float scan[360];
        ufr_recv(&scan_sub);
        ufr_copy_af32(&scan_sub, 360, scan);
        //ufr_get(&scan_sub, "^af", 360, scan);

        printf("%f %f\n", p1,p2);

        /*
        Eigen::Vector2d odom_pos();
        lama::Pose2D odom(odom_pos,0);
        bool update = slam.enoughMotion(odom);
        if ( update ){
            lama::PointCloudXYZ::Ptr cloud(new lama::PointCloudXYZ);
            slam.update(cloud, odom, 0.0);
        }
        */
    }

    return 0;
}
