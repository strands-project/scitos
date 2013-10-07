#include <stdio.h>
#include "OpenNI.h"
#include "ros/ros.h"

class ColorCallback : public openni::VideoStream::NewFrameListener
{
public:
    ColorCallback(bool publishRosMessage = true, bool createCVwin = false);
    void onNewFrame(openni::VideoStream& stream);
    void analyzeFrame(const openni::VideoFrameRef& frame);
    bool        saveOneFrame, saveFrameSequence,publishRosMessage, createCVWindow;
private:
    openni::VideoFrameRef m_frame;
    ros::NodeHandle       m_RosNode;
    ros::Publisher        m_RosImagePublisher;
    ros::Publisher        m_RosCameraInfoPublisher;
};
