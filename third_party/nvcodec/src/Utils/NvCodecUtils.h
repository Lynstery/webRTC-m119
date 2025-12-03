/*
 * This copyright notice applies to this header file only:
 *
 * Copyright (c) 2010-2024 NVIDIA Corporation
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the software, and to permit persons to whom the
 * software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

//---------------------------------------------------------------------------
//! \file NvCodecUtils.h
//! \brief Miscellaneous classes and error checking functions.
//!
//! Used by Transcode/Encode samples apps for reading input files, mutithreading, performance measurement or colorspace conversion while decoding.
//---------------------------------------------------------------------------

#pragma once
#include <chrono>
#include <sys/stat.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "unistd.h"
#include <vector>

#define MAKE_FOURCC( ch0, ch1, ch2, ch3 )                               \
                ( (uint32_t)(uint8_t)(ch0) | ( (uint32_t)(uint8_t)(ch1) << 8 ) |    \
                ( (uint32_t)(uint8_t)(ch2) << 16 ) | ( (uint32_t)(uint8_t)(ch3) << 24 ) )

inline void NvSleep(unsigned int mSec){
#if defined  WIN32 || defined _WIN32
    Sleep(mSec);
#else
    usleep(mSec * 1000);
#endif
}

inline bool check(int e, int iLine, const char *szFile) {
    if (e < 0) {
        return false;
    }
    return true;
}

#define ck(call) check(call, __LINE__, __FILE__)

/**
* @brief Utility class to measure elapsed time in seconds between the block of executed code
*/
class StopWatch {
public:
    void Start() {
        t0 = std::chrono::high_resolution_clock::now();
    }
    double Stop() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch() - t0.time_since_epoch()).count() / 1.0e9;
    }

private:
    std::chrono::high_resolution_clock::time_point t0;
};

/**
* @brief Class for writing IVF format header for AV1 codec
*/
class IVFUtils {
public:
    void WriteFileHeader(std::vector<uint8_t> &vPacket, uint32_t nFourCC, uint32_t nWidth, uint32_t nHeight, uint32_t nFrameRateNum, uint32_t nFrameRateDen, uint32_t nFrameCnt)
    {
        char header[32];

        header[0] = 'D';
        header[1] = 'K';
        header[2] = 'I';
        header[3] = 'F';
        mem_put_le16(header + 4, 0);                    // version
        mem_put_le16(header + 6, 32);                   // header size
        mem_put_le32(header + 8, nFourCC);              // fourcc
        mem_put_le16(header + 12, nWidth);              // width
        mem_put_le16(header + 14, nHeight);             // height
        mem_put_le32(header + 16, nFrameRateNum);       // rate
        mem_put_le32(header + 20, nFrameRateDen);       // scale
        mem_put_le32(header + 24, nFrameCnt);           // length
        mem_put_le32(header + 28, 0);                   // unused

        vPacket.insert(vPacket.end(), &header[0], &header[32]);
    }
    
    void WriteFrameHeader(std::vector<uint8_t> &vPacket,  size_t nFrameSize, int64_t pts)
    {
        char header[12];
        mem_put_le32(header, (int)nFrameSize);
        mem_put_le32(header + 4, (int)(pts & 0xFFFFFFFF));
        mem_put_le32(header + 8, (int)(pts >> 32));
        
        vPacket.insert(vPacket.end(), &header[0], &header[12]);
    }
    
private:
    static inline void mem_put_le32(void *vmem, int val)
    {
        unsigned char *mem = (unsigned char *)vmem;
        mem[0] = (unsigned char)((val >>  0) & 0xff);
        mem[1] = (unsigned char)((val >>  8) & 0xff);
        mem[2] = (unsigned char)((val >> 16) & 0xff);
        mem[3] = (unsigned char)((val >> 24) & 0xff);
    }

    static inline void mem_put_le16(void *vmem, int val)
    {
        unsigned char *mem = (unsigned char *)vmem;
        mem[0] = (unsigned char)((val >>  0) & 0xff);
        mem[1] = (unsigned char)((val >>  8) & 0xff);
    }

};
