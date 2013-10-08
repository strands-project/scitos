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

#include "OniSampleUtilities.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

 #include "ros/ros.h"

using namespace openni;
using namespace std;
using namespace cv;

class OpenNIDeviceListener : public OpenNI::DeviceConnectedListener,
                                    public OpenNI::DeviceDisconnectedListener,
                                    public OpenNI::DeviceStateChangedListener
{
public:
    virtual void onDeviceStateChanged(const DeviceInfo* pInfo, DeviceState state)
    {
        printf("Device \"%s\" error state changed to %d\n", pInfo->getUri(), state);
    }

    virtual void onDeviceConnected(const DeviceInfo* pInfo)
    {
        printf("Device \"%s\" connected\n", pInfo->getUri());
    }

    virtual void onDeviceDisconnected(const DeviceInfo* pInfo)
    {
        printf("Device \"%s\" disconnected\n", pInfo->getUri());
    }
};

int main(int argc, char** argv)
{
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 1;
    }

    OpenNIDeviceListener devicePrinter;

    OpenNI::addDeviceConnectedListener(&devicePrinter);
    OpenNI::addDeviceDisconnectedListener(&devicePrinter);
    OpenNI::addDeviceStateChangedListener(&devicePrinter);

    openni::Array<openni::DeviceInfo> deviceList;
    openni::OpenNI::enumerateDevices(&deviceList);
    for (int i = 0; i < deviceList.getSize(); ++i)
    {
        printf("Device \"%s\" already connected\n", deviceList[i].getUri());
    }

    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 2;
    }

    cout<<"This device supports the following sensors ";
    bool sensor = device.hasSensor(SENSOR_IR); if (sensor) cout <<"IR ";
    sensor = device.hasSensor(SENSOR_COLOR); if (sensor) cout <<"COLOR ";
    sensor = device.hasSensor(SENSOR_DEPTH); if (sensor) cout <<"DEPTH ";
    cout<<endl;

    ImageRegistrationMode mode = IMAGE_REGISTRATION_DEPTH_TO_COLOR;
    bool registrationSupported = device.getImageRegistrationMode();
    if(registrationSupported)
    {
        cout<<"Image registration SUPPORTED"<<endl;
        rc = device.setImageRegistrationMode(mode);
        // handle ret
        if (rc != STATUS_OK)
        {
            std::cout<<"Could not set the image registration on. Some error occured  "<<rc<<std::endl;
        }
    } else {
        cout<<"Image registration NOT SUPPORTED"<<endl;
    }

    rc = device.setDepthColorSyncEnabled(true);
    // handle rc
    if (rc != STATUS_OK)
    {
        std::cout<<"Could not set the depth-color sync. Some error occured"<<std::endl;
    }

    VideoStream depth;
    if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc != STATUS_OK)
        {
            printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        }
    }
    VideoMode depthVideoMode = depth.getVideoMode();
    depthVideoMode.setResolution(640,480);
    rc = depth.setVideoMode(depthVideoMode);
    if (rc != STATUS_OK)
    {
        printf("Couldn't set increased resolution for depth stream\n%s\n", OpenNI::getExtendedError());
    }

    rc = depth.start();
    if (rc != STATUS_OK)
    {
        printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
    }

    VideoStream color;
    if (device.getSensorInfo(SENSOR_COLOR) != NULL)
    {
        rc = color.create(device, SENSOR_COLOR);
        if (rc != STATUS_OK)
        {
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
        }
    }
    VideoMode colorVideoMode = color.getVideoMode();
    colorVideoMode.setResolution(640,480);
    rc = color.setVideoMode(colorVideoMode);
    if (rc != STATUS_OK)
    {
        printf("Couldn't set increased resolution for color stream\n%s\n", OpenNI::getExtendedError());
    }

    rc = color.start();
    if (rc != STATUS_OK)
    {
        printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
    }


    // initialize ros system
    ros::init(argc,argv, "openni_wrapper");



    DepthCallback aDepthCallback(true,false);
    // Register to new frame
    depth.addNewFrameListener(&aDepthCallback);

    ColorCallback aColorCallback(true,false);
    color.addNewFrameListener(&aColorCallback);


    ros::Rate loop_rate(10);
    // Wait while we're getting frames through the printer
    bool loop = true;
    while (loop && ros::ok())
    {
//        Sleep(100);
  //      sleep(1);
        int ch = wasKeyboardHit();

        switch (ch)
        {
            case 113: // key q
            {
                std::cout<<"------------------------- Exitting program -----------------------------"<<std::endl;
                loop = false;
                break;
            }

            case 115: // key s
            {
                std::cout<<"------------------------- Saving a frame -----------------------------"<<std::endl;
                aDepthCallback.saveOneFrame = true;
                aColorCallback.saveOneFrame = true;
                break;
            }
            case 97: // key a
            {
                std::cout<<"------------------------- Saving frame sequence-----------------------------"<<std::endl;
                aDepthCallback.saveFrameSequence= true;
                aColorCallback.saveFrameSequence = true;
                break;
            }
            case 122: // key z
            {
                std::cout<<"------------------------- Stopping saving -----------------------------"<<std::endl;
                aDepthCallback.saveFrameSequence = false;
                aColorCallback.saveFrameSequence = false;
                break;
            }

            case 0:
                break;
            default:
                   break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    depth.removeNewFrameListener(&aDepthCallback);
    color.removeNewFrameListener(&aColorCallback);


    depth.stop();
    depth.destroy();
    color.stop();
    color.destroy();
    device.close();
    OpenNI::shutdown();

    return 0;
}
