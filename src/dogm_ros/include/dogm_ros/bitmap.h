//
// Created by bjgilhul on 6/3/21.
//

#ifndef DOGM_ROS_BITMAP_H
#define DOGM_ROS_BITMAP_H

#include <fstream>
#include <ostream>
#include <vector>

//
// Borrowed and adapted from:
//
//  https://dev.to/muiz6/c-how-to-write-a-bitmap-image-from-scratch-1k6m
namespace dogm_ros {

struct BmpHeader {
    char bitmapSignatureBytes[2];
    uint32_t sizeOfBitmapFile;
    uint32_t reservedBytes;
    uint32_t pixelDataOffset;
};

struct BmpInfo {
    uint32_t sizeOfThisHeader;
    int32_t width;   // in pixels
    int32_t height;  // in pixels
    union {
        uint32_t colours;
        struct {
            uint16_t numberOfColorPlanes;  // must be 1
            uint16_t colorDepth;
        };
    };
    uint32_t compressionMethod = 0;
    uint32_t rawBitmapDataSize = 0;       // generally ignored
    int32_t horizontalResolution = 1180;  // in pixel per meter
    int32_t verticalResolution = 1180;    // in pixel per meter
    uint32_t colorTableEntries = 0;
    uint32_t importantColors = 0;
};

struct Pixel {
    char r;
    char g;
    char b;
};

const uint32_t bitmap_header_size = 14 + sizeof(BmpInfo);

class Bitmap {
public:
    Bitmap(uint32_t size)
        : header({{'B', 'M'}, bitmap_header_size + size * size, 0, bitmap_header_size}), data(size * size, {0, 0, 0})
    {
        info.sizeOfThisHeader = 40;
        info.width = int(size);
        info.height = int(size);
        info.numberOfColorPlanes = 1;
        info.colorDepth = 24;
    }

    Pixel& operator[](std::size_t idx) { return data[idx]; }

    const Pixel& operator[](std::size_t idx) const { return data[idx]; }

    void save(std::string filename)
    {
        std::ofstream f(filename, std::ios::binary);

        f.write((char*)&header.bitmapSignatureBytes, 2);
        f.write((char*)&header.sizeOfBitmapFile, 4);
        f.write((char*)&header.reservedBytes, 4);
        f.write((char*)&header.pixelDataOffset, 4);
        f.write((char*)&info, 40);
        for (const auto& pxl : data) {
            f.write((char*)&pxl, 3);
        }
        uint32_t null = 0;
        f.write((char*)&null, sizeof(null));
        f.write((char*)&null, sizeof(null));

        f.close();
    }

private:
    BmpHeader header;
    BmpInfo info;
    std::vector<Pixel> data;
};

}  // namespace dogm_ros

#endif  // DOGM_ROS_BITMAP_H
