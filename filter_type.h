#ifndef FILTER_TYPE_H
#define FILTER_TYPE_H

typedef enum {
    FILTER_TYPE_NOT_SET = 0,
    LPF_ORDER_1,
    LPF_ORDER_2,
    LPF_ORDER_3,
    LPF_ORDER_4,
    HPF_ORDER_1,
    HPF_ORDER_2,
    HPF_ORDER_3,
    HPF_ORDER_4,
    LOW_SHELF,
    HIGH_SHELF,
    PEAK,
    NOTCH,
    BAND_PASS
} FilterType;

#endif
