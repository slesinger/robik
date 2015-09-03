#include "robik_driver.h"

#define DEG_INFINITY 721

double map_unchecked(double value, double fromLow, double fromHigh, double toLow, double toHigh) {

  if (fromHigh == fromLow)
    return fromLow;

  return (value-fromLow) * ( (toHigh-toLow) / (fromHigh-fromLow) )+toLow;
}

double map_check_inf(double value, double fromLow, double fromHigh, double toLow, double toHigh) {

    float val = map_unchecked(value, fromLow, fromHigh, toLow, toHigh);
    if (val < toLow) val = DEG_INFINITY;
    if (val > toHigh) val = DEG_INFINITY;
    return val;
}

double map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {

    double val = map_unchecked(value, fromLow, fromHigh, toLow, toHigh);
    if (val < toLow) val = toLow;
    if (val > toHigh) val = toHigh;
    return val;
}
