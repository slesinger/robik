#include "robik_driver.h"
#include "robik_util.h"

double map_unchecked(double value, double fromLow, double fromHigh, double toLow, double toHigh) {

  if (fromHigh == fromLow)
    return fromLow;

  return (value-fromLow) * ( (toHigh-toLow) / (fromHigh-fromLow) )+toLow;
}

double map_check_inf(double value, double fromLow, double fromHigh, double toLow, double toHigh) {

    float val = map_unchecked(value, fromLow, fromHigh, toLow, toHigh);
    if (val < toLow) val = toLow;
    if (val > toHigh) val = toHigh;
    return val;
}

double map_avgcheck_inf(double value, t_parm *parm) {

    float val = map_unchecked(value, parm->fromLow, parm->fromHigh, parm->toLow, parm->toHigh);

    //check if new value is not too far away from last
    if ( (abs(val - parm->avg3) < parm->threshold) || (parm->outlier_cnt > parm->outlier_cnt_max) ) {

	if (val < parm->toLow) val = parm->toLow;
	if (val > parm->toHigh) val = parm->toHigh;

        parm->avg1 = parm->avg2;
        parm->avg2 = parm->avg3;
        parm->avg3 = val;
	parm->outlier_cnt = 0;
    }
    else {
	parm->outlier_cnt++;
    }
    return (parm->avg1 + parm->avg2 + parm->avg3) / 3;
}

double map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {

    double val = map_unchecked(value, fromLow, fromHigh, toLow, toHigh);
    if (val < toLow) val = toLow;
    if (val > toHigh) val = toHigh;
    return val;
}
