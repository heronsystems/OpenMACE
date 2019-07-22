#ifndef MATH_HELPER_H
#define MATH_HELPER_H

#include <stdint.h>

class MathHelper {

public:

    template <typename T>
    static uint8_t calc_checksum(const T *buf, const size_t start, const size_t end) {
        static_assert(sizeof(T) == 1, "Must be a byte");

        uint64_t sum = 0;
        for (size_t i = start; i < end; i++) {
            sum += buf[i];
        }
        return (0xff - sum) & 0xff;
    }
};

#endif // MATH_HELPER_H
