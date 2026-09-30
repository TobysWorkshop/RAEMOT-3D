#pragma once

// File format for the stereo 3D track file //

// The file can natively be read with np.fromfile / nmap, no parsing required
// Records from different tracks are interleaved in arrival order. They need to be grouped by gloab_id downstream for any visualisation or processing
// Per track, the evolution is: OPEN, POINT, POINT, ..., CLOSE. A track that is still alive when the system is stopped has no CLOSE entry.
// global_id is 32-bit, and as such can hold up to 4 billion tracks per file. Should be enough.

// For a numpy reader:
// rec = np.dtype([('type', 'u1'),('pad','V3'),('id','<u4'),('tg','<f8'),('p','<f8',6)])
// raw = np.fromfile(path, dtype=rec, offset=64)
// pts = raw[raw['type'] == 1]  # p = x, y, z, vx, vy, vz
// opn = raw[raw['type'] == 2]  # opn['p'].view('<i4')[:, :2] = (a_track_id, b_track_id)

#include <cstdint>
#include <type_traits>

namespace raemot3d_file {
    constexpr char MAGIC[8] = {'R', 'A', 'E', 'M', 'O', 'T', '3', 'D'};
    constexpr uint32_t VERSION = 1;

    constexpr uint8_t REC_INVALID = 0;
    constexpr uint8_t REC_POINT = 1;    // one tringulated 3D state
    constexpr uint8_t REC_OPEN = 2;     // when track gets validated - tells you which ids in each camera map to each other
    constexpr uint8_t REC_CLOSE = 3;    // when track ends

    struct alignas(64) FileHeader {
        char magic[8];
        uint32_t version;
        uint32_t record_size;
        uint64_t created_unix_ns;
        uint8_t reserved[40];
    };
    static_assert(sizeof(FileHeader) == 64, "FileHeader must by 64 bytes!");

    struct PointPayload { double x, y, z, vx, vy, vz; };
    struct OpenPayload { 
        int32_t a_track_id; 
        int32_t b_track_id; 
        uint8_t reserved[40]; 
    };
    struct ClosePayload { uint8_t reserved[48]; };

    struct alignas(64) Record {
        uint8_t type;
        uint8_t reserved[3];
        uint32_t global_id;
        double tg;
        union Payload {
            PointPayload point;
            OpenPayload open;
            ClosePayload close;
        } payload;
    };
    static_assert(sizeof(Record) == 64, "Record must be exactly one cache line (64 bytes!)");
    static_assert(std::is_trivially_copyable_v<Record>, "Record must be trivially copyable!");

} // end namespace raoemot3d_file