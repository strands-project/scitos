/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "OpenNI.h"
#include "ColorCallback.h"
#include "DepthCallback.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

 #include "ros/ros.h"

#include "OpenniWrapperNode.h"

using namespace openni;
using namespace std;
using namespace cv;



int main(int argc, char** argv)
{



    ros::init(argc,argv, "openni_wrapper");

    ros::NodeHandle private_node_handle_("~");
    ros::NodeHandle aRosNode;

    OpenniWrapperNode node(aRosNode, private_node_handle_);

    node.initializeOpenni();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    node.terminateOpenni();

    return 0;
}
