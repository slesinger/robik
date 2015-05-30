#include "robik_driver.h"

#define DEG_INFINITY 721

float map_unchecked(float value, float fromLow, float fromHigh, float toLow, float toHigh) {

  if (fromHigh == fromLow)
    return fromLow;

  return (value-fromLow) * ( (toHigh-toLow) / (fromHigh-fromLow) )+toLow;
}

float map_check_inf(float value, float fromLow, float fromHigh, float toLow, float toHigh) {

    float val = map_unchecked(value, fromLow, fromHigh, toLow, toHigh);
    if (val < toLow) val = DEG_INFINITY;
    if (val > toHigh) val = DEG_INFINITY;
    return val;
}

float map(float value, float fromLow, float fromHigh, float toLow, float toHigh) {

    float val = map_unchecked(value, fromLow, fromHigh, toLow, toHigh);
    if (val < toLow) val = toLow;
    if (val > toHigh) val = toHigh;
    return val;
}
