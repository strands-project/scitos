#include "OpenniWrapperNode.h"

using namespace openni;
using namespace std;
using namespace cv;

OpenniWrapperNode::OpenniWrapperNode(ros::NodeHandle nh, ros::NodeHandle private_nh): m_ColorCallback(nh, true, false), m_DepthCallback(nh, true, false)
{
    m_nodeHandle = nh;
    m_privateNodeHandle = private_nh;
}

OpenniWrapperNode::~OpenniWrapperNode()
{

}


void OpenniWrapperNode::initializeOpenni()
{
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 1;
    }



    OpenNI::addDeviceConnectedListener(&m_devicePrinter);
    OpenNI::addDeviceDisconnectedListener(&m_devicePrinter);
    OpenNI::addDeviceStateChangedListener(&m_devicePrinter);

    openni::Array<openni::DeviceInfo> deviceList;
    openni::OpenNI::enumerateDevices(&deviceList);
    for (int i = 0; i < deviceList.getSize(); ++i)
    {
        printf("Device \"%s\" already connected\n", deviceList[i].getUri());
    }


    rc = m_Device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 2;
    }

    cout<<"This device supports the following sensors ";
    bool sensor = m_Device.hasSensor(SENSOR_IR); if (sensor) cout <<"IR ";
    sensor = m_Device.hasSensor(SENSOR_COLOR); if (sensor) cout <<"COLOR ";
    sensor = m_Device.hasSensor(SENSOR_DEPTH); if (sensor) cout <<"DEPTH ";
    cout<<endl;

    ImageRegistrationMode mode = IMAGE_REGISTRATION_DEPTH_TO_COLOR;
    bool registrationSupported = m_Device.getImageRegistrationMode();
    if(registrationSupported)
    {
        cout<<"Image registration SUPPORTED"<<endl;
        rc = m_Device.setImageRegistrationMode(mode);
        // handle ret
        if (rc != STATUS_OK)
        {
            std::cout<<"Could not set the image registration on. Some error occured  "<<rc<<std::endl;
        }
    } else {
        cout<<"Image registration NOT SUPPORTED"<<endl;
    }

    rc = m_Device.setDepthColorSyncEnabled(true);
    // handle rc
    if (rc != STATUS_OK)
    {
        std::cout<<"Could not set the depth-color sync. Some error occured"<<std::endl;
    }

    if (m_Device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
        rc = m_Depth.create(m_Device, SENSOR_DEPTH);
        if (rc != STATUS_OK)
        {
            printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        }
    }
    VideoMode depthVideoMode = m_Depth.getVideoMode();
    depthVideoMode.setResolution(640,480);
    rc = m_Depth.setVideoMode(depthVideoMode);
    if (rc != STATUS_OK)
    {
        printf("Couldn't set increased resolution for depth stream\n%s\n", OpenNI::getExtendedError());
    }

    rc = m_Depth.start();
    if (rc != STATUS_OK)
    {
        printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
    }

    if (m_Device.getSensorInfo(SENSOR_COLOR) != NULL)
    {
        rc = m_Color.create(m_Device, SENSOR_COLOR);
        if (rc != STATUS_OK)
        {
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
        }
    }
    VideoMode colorVideoMode = m_Color.getVideoMode();
    colorVideoMode.setResolution(640,480);
    rc = m_Color.setVideoMode(colorVideoMode);
    if (rc != STATUS_OK)
    {
        printf("Couldn't set increased resolution for color stream\n%s\n", OpenNI::getExtendedError());
    }

    rc = m_Color.start();
    if (rc != STATUS_OK)
    {
        printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
    }


    // Register to new frame
    m_Depth.addNewFrameListener(&m_DepthCallback);

    m_Color.addNewFrameListener(&m_ColorCallback);

    std::string camNamespace;

    m_privateNodeHandle.getParam("camera", camNamespace);
    std::cout<<"Camera TF namespace is "<<camNamespace<<std::endl;

    m_DepthCallback.m_CameraNamespace = camNamespace;
    m_ColorCallback.m_CameraNamespace = camNamespace;


}

void OpenniWrapperNode::terminateOpenni()
{
    std::cout<<"Shutting down Openni driver "<<std::endl;

    m_Depth.removeNewFrameListener(&m_DepthCallback);
    m_Color.removeNewFrameListener(&m_ColorCallback);

    m_Depth.stop();
    m_Depth.destroy();
    m_Color.stop();
    m_Color.destroy();
    m_Device.close();
    OpenNI::shutdown();
}
