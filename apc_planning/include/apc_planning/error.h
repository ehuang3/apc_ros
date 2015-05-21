#pragma once
#include <stdexcept>
#include <cstdio>
#include <string.h>
#include <signal.h>
#include <execinfo.h>
#include <stdlib.h>
#include <apc_msgs/PrimitivePlan.h>

#define __APC_FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

enum APC_ERROR
{
    NO_ERROR,
    COLLISION_IN_TRAJECTORY,
    NO_PLAN,
};

namespace apc_exception
{
    std::string GetResolvedStackTrace();
    std::string show_backtrace();

    struct Exception : public std::runtime_error
    {
        Exception(const std::string& what)
            : std::runtime_error(what)
        {}
        Exception(const std::string& what, const apc_msgs::PrimitivePlan& plan)
            : std::runtime_error(what),
              what_plan(plan)
        {}
        virtual ~Exception() throw(){}
        apc_msgs::PrimitivePlan what_plan;
    };
}

#define APC_ASSERT_PLAN_VECTOR(x, plans, ...)                           \
    if (!(x))                                                           \
    {                                                                   \
        std::string bt = apc_exception::GetResolvedStackTrace();        \
        char buf[4096];                                                 \
        snprintf(buf, 4096, __VA_ARGS__);                               \
        char buf2[4096];                                                \
        snprintf(buf2, 4096, "in %s of %s line %d\n",                   \
                 __FUNCTION__, __APC_FILENAME__, __LINE__);             \
        std::string error = std::string(buf2) +                         \
            std::string(buf) + std::string("\n") + bt;                  \
        static int fi = 0;                                              \
        throw apc_exception::Exception(error,                           \
                                       plans[(fi++) % plans.size()]);   \
    }                                                                   \

#define APC_ASSERT_PLAN(x, plan, ...)                                   \
    if (!(x))                                                           \
    {                                                                   \
        std::string bt = apc_exception::GetResolvedStackTrace();        \
        char buf[4096];                                                 \
        snprintf(buf, 4096, __VA_ARGS__);                               \
        char buf2[4096];                                                \
        snprintf(buf2, 4096, "in %s of %s line %d\n",                   \
                 __FUNCTION__, __APC_FILENAME__, __LINE__);             \
        std::string error = std::string(buf2) +                         \
            std::string(buf) + std::string("\n") + bt;                  \
        throw apc_exception::Exception(error, plan);                    \
    }                                                                   \

#define APC_ASSERT(x, ...)                                              \
    if (!(x))                                                           \
    {                                                                   \
        std::string bt = apc_exception::GetResolvedStackTrace();        \
        char buf[4096];                                                 \
        snprintf(buf, 4096, __VA_ARGS__);                               \
        char buf2[4096];                                                \
        snprintf(buf2, 4096, "in %s of %s line %d\n",                   \
                 __FUNCTION__, __APC_FILENAME__, __LINE__);             \
        std::string error = std::string(buf2) +                         \
            std::string(buf) + std::string("\n") + bt;                  \
        throw apc_exception::Exception(error);                          \
    }                                                                   \

#define HELLO_WORLD
