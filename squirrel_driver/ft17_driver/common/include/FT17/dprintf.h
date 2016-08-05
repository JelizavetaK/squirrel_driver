#ifndef __DPRINTF_H__
#define __DPRINTF_H__

#ifdef __XENO__
    #include <rtdk.h>
    #include <rtnet.h>
    #define DPRINTF rt_printf
#else
    #include <stdio.h>
    #define DPRINTF printf
#endif

#endif

