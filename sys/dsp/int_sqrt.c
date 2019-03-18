#include "dsp/int_sqrt.h"

uint32_t int_sqrt_32(uint32_t n)
{
    register uint32_t root, remainder, place;

    root = 0;
    remainder = n;
    place = 0x40000000;

    while (place > remainder) {
        place = place >> 2;
    }
    while (place) {
        if (remainder >= root + place) {
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root = root >> 1;
        place = place >> 2;
    }
    return root;
}

uint16_t int_sqrt_16(uint16_t n)
{
    register uint16_t root, remainder, place;

    root = 0;
    remainder = n;
    place = 0x4000;

    while (place > remainder) {
        place = place >> 2;
    }
    while (place) {
        if (remainder >= root + place) {
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root = root >> 1;
        place = place >> 2;
    }
    return root;
}

uint8_t int_sqrt_8(uint8_t n)
{
    register uint8_t root, remainder, place;

    root = 0;
    remainder = n;
    place = 0x40;

    while (place > remainder) {
        place = place >> 2;
    }
    while (place) {
        if (remainder >= root + place) {
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root = root >> 1;
        place = place >> 2;
    }
    return root;
}