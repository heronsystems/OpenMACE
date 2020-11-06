#include "empty_mavlink.h"

namespace DataMAVLINK {

const char EmptyMAVLINK_name[] = "emptyMAVLINK";
const MaceCore::TopicComponentStructure EmptyMAVLINK_structure = []{
    MaceCore::TopicComponentStructure structure;
    return structure;
}();

MaceCore::TopicDatagram EmptyMAVLINK::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;
    return datagram;
}

void EmptyMAVLINK::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    UNUSED(datagram);
}

EmptyMAVLINK::EmptyMAVLINK()
{

}

} //end of namespace DataMAVLINK
