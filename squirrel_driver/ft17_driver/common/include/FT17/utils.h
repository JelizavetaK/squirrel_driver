#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>

#ifdef __XENO__
#include <rtdk.h>
#include <rtnet.h>
#define DPRINTF rt_printf
#else
#include <stdio.h>
#define DPRINTF printf
#endif

#define DEG2mRAD(X) (X*M_PI*1e5)/180
#define DEG2RAD(X)  (X*M_PI)/180

#define mRAD2DEG(X) (X*180)/(M_PI*1e5)

#define NSEC_PER_SEC	1000000000ULL

inline uint64_t get_time_ns ( clockid_t clock_id=CLOCK_MONOTONIC )
{
        uint64_t time_ns;
        struct timespec ts;
        clock_gettime ( clock_id, &ts );
        time_ns = ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
        return time_ns;
}

inline void tsnorm ( struct timespec *ts )
{
        while ( ts->tv_nsec >= NSEC_PER_SEC ) {
                ts->tv_nsec -= NSEC_PER_SEC;
                ts->tv_sec++;
        }
}

#endif
