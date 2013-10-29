#ifndef __OPENNI_WRAPPER_NODE__
#define __OPENNI_WRAPPER_NODE__

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "OpenNI.h"
#include "ColorCallback.h"
#include "DepthCallback.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

 #include "ros/ros.h"
class OpenNIDeviceListener : public openni::OpenNI::DeviceConnectedListener,
                                    public openni::OpenNI::DeviceDisconnectedListener,
                                    public openni::OpenNI::DeviceStateChangedListener
{
public:
    virtual void onDeviceStateChanged(const openni::DeviceInfo* pInfo, openni::DeviceState state)
    {
        printf("Device \"%s\" error state changed to %d\n", pInfo->getUri(), state);
    }

    virtual void onDeviceConnected(const openni::DeviceInfo* pInfo)
    {
        printf("Device \"%s\" connected\n", pInfo->getUri());
    }

    virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo)
    {
        printf("Device \"%s\" disconnected\n", pInfo->getUri());
    }
};


class OpenniWrapperNode
{
public:
    OpenniWrapperNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~OpenniWrapperNode();

    void initializeOpenni();
    void terminateOpenni();

private:
    ros::NodeHandle m_nodeHandle;
    ros::NodeHandle m_privateNodeHandle;

    OpenNIDeviceListener m_devicePrinter;
    openni::Device m_Device;
    openni::VideoStream m_Color;
    openni::VideoStream m_Depth;
    ColorCallback       m_ColorCallback;
    DepthCallback       m_DepthCallback;




};


#endif
