#ifndef COMMAND_VALIDITY_TYPE_H
#define COMMAND_VALIDITY_TYPE_H

#include <string>
#include <stdexcept>

namespace Data
{

enum class CommandValidityType{
    CV_VALID=0, /* Command / mission item is ok. | */
    CV_INVALID=1, /* Command / mission item is ok. | */
    CV_UNKNOWN=2 /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | */
};

} //end of namespace Data

#endif // COMMAND_VALIDITY_TYPE_H
