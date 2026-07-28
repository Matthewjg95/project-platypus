// object_labels.cpp
#include "object_labels.h"

namespace object_labels {

static const char* const kNames[80] = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra",
    "giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis",
    "snowboard","sports ball","kite","baseball bat","baseball glove",
    "skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork",
    "knife","spoon","bowl","banana","apple","sandwich","orange","broccoli",
    "carrot","hot dog","pizza","donut","cake","chair","couch","potted plant",
    "bed","dining table","toilet","tv","laptop","mouse","remote","keyboard",
    "cell phone","microwave","oven","toaster","sink","refrigerator","book",
    "clock","vase","scissors","teddy bear","hair drier","toothbrush",
};

const char* name(uint8_t class_id) {
    return (class_id < 80) ? kNames[class_id] : "obj";
}

// 12-entry palette; class ids map by modulo for stable, distinct colours.
static const uint32_t kPalette[12] = {
    0xE6194B, 0x3CB44B, 0xFFE119, 0x4363D8, 0xF58231, 0x911EB4,
    0x42D4F4, 0xF032E6, 0xBFEF45, 0xFABED4, 0x469990, 0xDCBEFF,
};

uint32_t color(uint8_t class_id) {
    return kPalette[class_id % 12];
}

} // namespace object_labels
