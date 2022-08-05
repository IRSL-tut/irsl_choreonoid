#ifndef __IRSL_DEBUG_H__
#define __IRSL_DEBUG_H__

#ifdef IRSL_DEBUG
#include <iostream>
#define DEBUG_SIMPLE_NL(args) std::cerr << args
#define DEBUG_SIMPLE(args) std::cerr << args << std::endl
#define DEBUG_STREAM_NL_NL(args) \
    std::cerr << "[" << __PRETTY_FUNCTION__ << "]" << args
#define DEBUG_STREAM_NL(args) \
    std::cerr << "[" << __PRETTY_FUNCTION__ << "]" << args << std::endl
#define DEBUG_PRINT() \
    std::cerr << "[" << __PRETTY_FUNCTION__ << "]" << std::endl
#else
#define DEBUG_SIMPLE_NL(args)
#define DEBUG_SIMPLE(args)
#define DEBUG_STREAM_NL_NL(args)
#define DEBUG_STREAM_NL(args)
#define DEBUG_PRINT(args)
#endif

#define INFO_SIMPLE_NL(args) std::cout << args
#define INFO_SIMPLE(args) std::cout << args << std::endl
#define INFO_STREAM_NL(args) \
    std::cout << "[" << __PRETTY_FUNCTION__ << "]" << args
#define INFO_STREAM(args) \
    std::cout << "[" << __PRETTY_FUNCTION__ << "]" << args << std::endl
#define INFO_PRINT() \
    std::cout << "[" << __PRETTY_FUNCTION__ << "]" << std::endl

#define ERROR_SIMPLE_NL(args) std::cerr << args
#define ERROR_SIMPLE(args) std::cerr << args << std::endl
#define ERROR_STREAM_NL(args) \
    std::cerr << "[" << __PRETTY_FUNCTION__ << "]" << args
#define ERROR_STREAM(args) \
    std::cerr << "[" << __PRETTY_FUNCTION__ << "]" << args << std::endl
#define ERROR_PRINT() \
    std::cerr << "[" << __PRETTY_FUNCTION__ << "]" << std::endl



#endif
