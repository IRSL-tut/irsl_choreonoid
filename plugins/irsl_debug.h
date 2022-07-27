#ifndef __IRSL_DEBUG_H__
#define __IRSL_DEBUG_H__

#ifdef IRSL_DEBUG
#include <iostream>
#define DEBUG_STREAM(args) std::cerr << args
#define DEBUG_STREAM_INFO(_cls,_mtd,args) \
    std::cerr << "[" #_cls "::" #_mtd "]" << args
#else
#define DEBUG_STREAM(args)
#define DEBUG_STREAM_INFO(_cls,_mtd,args)
#endif

#endif
