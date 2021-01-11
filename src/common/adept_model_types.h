#ifndef ADEPT_MODEL_TYPES_H
#define ADEPT_MODEL_TYPES_H
#include <stdint.h>
#include <ostream>

typedef enum class AdeptModelType : uint8_t
{
    ZOMBIE,
    CIRCLE,
    STATIC,
    FALCO,
    TALON,
    MANUAL,
    UNKNOWN
} AdeptModelType;

/*!
 * \brief Determines if the given model type is a Spectre Agent
 * \param type Model type
 * \return String representation
 */
inline bool ModelTypeIsSpectre(AdeptModelType type){
    switch (type)
    {
        case AdeptModelType::ZOMBIE:
            return true;
        case AdeptModelType::CIRCLE:
            return true;
        case AdeptModelType::STATIC:
            return true;
        default:
            return false;
    }
}
/*!
 * \brief Converts Agent Model type enum value to a string
 * \param type Model type
 * \return String representation
 */
inline std::string AdeptModelToString(AdeptModelType type)
{
    switch (type)
    {
    case AdeptModelType::STATIC:
        return "STATIC";
    case AdeptModelType::ZOMBIE:
        return "ZOMBIE";
    case AdeptModelType::CIRCLE:
        return "CIRCLE";
    case AdeptModelType::FALCO:
        return "FALCO";
    case AdeptModelType::TALON:
        return "TALON";
    case AdeptModelType::MANUAL:
        return "MANUAL";
    case AdeptModelType::UNKNOWN:
    default:
        return "Unknown";
    }
}

/*!
 * \brief Converts Agent Model type enum value to a string
 * \param type Model type
 * \return String representation
 */
inline AdeptModelType AdeptModelFromString(const std::string &type)
{
    if (type == "STATIC"){
        return AdeptModelType::STATIC;
    } else if (type == "ZOMBIE"){
        return AdeptModelType::ZOMBIE;
    } else if (type == "CIRCLE"){
        return AdeptModelType::CIRCLE;
    } else if (type == "FALCO"){
        return AdeptModelType::FALCO;
    } else if (type == "TALON"){
        return AdeptModelType::TALON;
    } else if (type == "MANUAL"){
        return AdeptModelType::MANUAL;
    } else {
        return AdeptModelType::UNKNOWN;
    }
}
#endif // ADEPT_MODEL_TYPES_H
