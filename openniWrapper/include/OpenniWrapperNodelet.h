#include <nodelet/nodelet.h>
#include "OpenniWrapperNode.h"

class OpenniWrapperNodelet: public nodelet::Nodelet
{

public:
    OpenniWrapperNodelet(){}
    ~OpenniWrapperNodelet(){}
    virtual void onInit();
    boost::shared_ptr<OpenniWrapperNode> inst_;

};

