#include <nodelet/nodelet.h>
#include "openniWrapperNode.h"

class OpenniWrapperNodelet: public nodelet::Nodelet
{

public:
    OpenniWrapperNodelet(){}
    ~OpenniWrapperNodelet(){}
    virtual void onInit();
    boost::shared_ptr<OpenniWrapperNode> inst_;

};

