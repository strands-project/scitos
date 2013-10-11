#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "OpenniWrapperNode.h"
#include "OpenniWrapperNodelet.h"


  void OpenniWrapperNodelet::onInit()
 {

   NODELET_DEBUG("Initializing nodelet");
   inst_.reset(new OpenniWrapperNode(getNodeHandle(),getPrivateNodeHandle()));
   inst_->initializeOpenni();
 }


  PLUGINLIB_DECLARE_CLASS(openni_wrapper,OpenniWrapperNodelet, OpenniWrapperNodelet, nodelet::Nodelet)
