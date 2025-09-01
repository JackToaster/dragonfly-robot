#pragma once
#include "ch32fun.h"



struct __attribute__((packed)) crsf_channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
    uint8_t armStatus; // optional ExpressLRS 4.0
};

// static void UnpackChannels(uint8_t const * const payload, uint32_t * const dest)
// {
//     const unsigned numOfChannels = 16;
//     const unsigned srcBits = 11;
//     const unsigned dstBits = 32;
//     const unsigned inputChannelMask = (1 << srcBits) - 1;

//     // code from BetaFlight rx/crsf.cpp / bitpacker_unpack
//     uint8_t bitsMerged = 0;
//     uint32_t readValue = 0;
//     unsigned readByteIndex = 0;
//     for (uint8_t n = 0; n < numOfChannels; n++)
//     {
//         while (bitsMerged < srcBits)
//         {
//             uint8_t readByte = payload[readByteIndex++];
//             readValue |= ((uint32_t) readByte) << bitsMerged;
//             bitsMerged += 8;
//         }
//         //printf("rv=%x(%x) bm=%u\n", readValue, (readValue & inputChannelMask), bitsMerged);
//         dest[n] = (readValue & inputChannelMask);
//         readValue >>= srcBits;
//         bitsMerged -= srcBits;
//     }
// }

