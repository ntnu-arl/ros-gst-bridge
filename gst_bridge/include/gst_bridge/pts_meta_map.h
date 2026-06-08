#ifndef PTS_META_MAP_H
#define PTS_META_MAP_H

#include <map>
#include <mutex>
#include <gst/gst.h>

struct FrameMetaData {
    guint64 argusTimestamp;
    guint64 exposureTime;
    gfloat analogGain;
    gfloat digitalGain;
};

class PtsMetaMap {
public:
    // Only declare it here, don't define it
    static PtsMetaMap& getInstance();
    
    void addMeta(guint64 pts, FrameMetaData meta);
    bool getAndRemoveMeta(guint64 pts, FrameMetaData& meta);

private:
    PtsMetaMap() {}
    std::map<guint64, FrameMetaData> meta_map_;
    std::mutex mtx_;
};

#endif // PTS_META_MAP_H