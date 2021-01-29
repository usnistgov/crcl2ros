// Core.h

/*
* DISCLAIMER:
* This software was produced by the National Institute of Standards
* and Technology (NIST), an agency of the U.S. government, and by statute is
* not subject to copyright in the United States.  Recipients of this software
* assume all responsibility associated with its operation, modification,
* maintenance, and subsequent redistribution.
*
* See NIST Administration Manual 4.09.07 b and Appendix I.
*/

#ifndef _CORE_H_
#define _CORE_H_

// C++ headers
#define _USE_MATH_DEFINES
#include <math.h>       /* isnan, sqrt */
#include <stdarg.h>
#include <memory>
#include <stdio.h>
#include <stdarg.h>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <sstream>
#include <cstring>
#include <ctime>
#include <mutex>
#include <thread>

#ifndef DEBUG
#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/null.hpp"
#endif

// boost headers

#include <boost/thread.hpp>
#include <boost/preprocessor.hpp>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>

#include <tf/tf.h>

#include <sys/syscall.h>

#if 0
template <typename T>
class Property {
public:
    virtual ~Property() {}
    virtual T & operator = (const T &f) { return value = f; }
    virtual operator T const & () const { return value; }
protected:
    T value;
};

To define a property:

Property<float> x;

To implement a custom getter/setter just inherit:

class : public Property<float> {
    virtual float & operator = (const float &f) { /*custom code*/ return value = f; }
    virtual operator float const & () const { /*custom code*/ return value; }
} y;
#endif
/**
 * @brief The Threads class
 */
class Threads
{
public:
    static std::map<std::thread::id, pid_t> & threads()
    {
        static std::map<std::thread::id, pid_t> THREADS;
        return THREADS;
    }
    static std::mutex & M()
    {
        static std::mutex m;
        return m;
    }
    static void add_pid_mapping()
    {
        std::lock_guard<std::mutex> l(M());
        threads()[std::this_thread::get_id()] = syscall(SYS_gettid);
    }
    //https://www.ibm.com/support/knowledgecenter/en/SSLTBW_2.3.0/com.ibm.zos.v2r3.bpxbd00/ptkill.htm
//    static bool is_killed(int threadid)
//    {
//        int status = pthread_kill( threadid, SIGUSR1);
//        if ( status <  0)
//            return 0;
//        return 1;
//    }
};

extern void add_pid_mapping();


/**
 * @brief ThreadingMutex - provide a global mutex to use in all threads.
 * Uses:
 * * ThreadingMutex().lock()/ThreadingMutex().unlock()
 * * std::lock_guard<std::mutex> lock(ThreadingMutex());
 * @return
 */
inline std::mutex & ThreadingMutex()
{
    //  A static local variable in an extern inline function always refers to the same object.  7.1.2/4 - C++98/C++14 (n3797)
    static std::mutex m;
    return m;
}

/**
   Turn string s into a vector of types T using separator
 */
template<class T>
inline void tokenizeV(const std::string &s,
               std::vector<T> &o, std::string separator)
{
    typedef boost::tokenizer<boost::char_separator<char> > tok_t;
    boost::char_separator<char> sep(separator.c_str());
    tok_t tok(s, sep);
    for (tok_t::iterator j (tok.begin());
         j != tok.end();
         ++j)
    {
        std::string f(*j);
        boost::trim(f);
        o.push_back(boost::lexical_cast<T>(f));
    }
}

/*!
* \brief StrFormat  accepts a traditional C format string and expects parameter to follow on calling stack and will
* produce a string from it.
* \param fmt is the C format string.
*/
#ifndef _STR_FORMAT_
#define _STR_FORMAT_

inline std::string StrFormat (const char *fmt, ...)
{
    va_list argptr;
    va_start(argptr, fmt);
    int m;
    int n = (int) strlen(fmt) + 1028;
    std::string tmp(n, '0');
    while ( ( m = vsnprintf(&tmp[0], n - 1, fmt, argptr) ) < 0 )
    {
        n = n + 1028;
        tmp.resize(n, '0');
    }
    va_end(argptr);
    return tmp.substr(0, m);
}
#endif
inline std::string getexepath()
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}
inline std::string getexefolder()
{
    std::string exepath = getexepath();
    return  exepath.substr(0, exepath.find_last_of('/') + 1);
}

#ifndef TODO
#define DO_PRAGMA(x) _Pragma (#x)
#define TODO(x) DO_PRAGMA(message ("TODO - " #x))
#endif      

/** \def TOOLS_DEPRECATED
    Macro that marks functions as deprecated -from ROS */

#ifdef __GNUC__
#define TOOLS_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define TOOLS_DEPRECATED __declspec(deprecated)
#elif defined(__clang__)
#define TOOLS_DEPRECATED __attribute__((deprecated("Use of this method is deprecated")))
#else
#define TOOLS_DEPRECATED /* Nothing */
#endif


#if defined DEBUG
#define IfDebug(arg)    arg
#else
#define IfDebug(arg)
#endif

// Adapter from  IKFAST
#ifndef NC_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif
#define NC_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "nc exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }
#endif



#ifndef CLEANSTORE
#define CLEANSTORE(Y, X, Z) \
    try{ Y = X; }           \
    catch ( ... ) { Y = Z; }
#define VALIDSTORE(Y, X) \
    try{ Y = X; }        \
    catch ( ... ) { }
#endif


#ifndef VAR

#define SINGLE_ARG(...) __VA_ARGS__

template <typename T, typename U>
size_t
offsetOf(U T::*member) {
    return (char *)&((T *)nullptr->*member) - (char *)nullptr;
}

inline std::vector<std::string>& var_breakpoint_list()
{
    static std::vector<std::string> list;
    return list;
}
inline void add_breakpoint_var(std::string varname)
{
    std::vector<std::string> & list (var_breakpoint_list());
    list.push_back(varname);
}
inline void remove_breakpoint_var(std::string varname)
{
    std::vector<std::string> & list (var_breakpoint_list());
    std::vector<std::string>::iterator it=std::find(list.begin(), list.end(), varname);
    if(it!=list.end())
        list.erase(it);
}
inline bool is_breakpoint_var(std::string varname)
{
    std::vector<std::string> & list (var_breakpoint_list());
    std::vector<std::string>::iterator it=std::find(list.begin(), list.end(), varname);
    if(it!=list.end())
        return true;
    return false;
}

#define VAR(Y, X)    \
    public: Y _ ## X; \
    public: Y & X( ) { \
    if(is_breakpoint_var(#X)) { assert(0); }\
    return _ ## X; }

#define STR_HELPER(x) #x

#define DVAR(C, X, Y)    \
    protected: Y _ ## X; \
    public: Y & X( ) { \
    return _ ## X; }

//constexpr static bool dummy ## _ ## X = []{ mClassNameVector()[STR_HELPER(_##X)]= offsetOf(&C::_##X) ; return true; };
//constexpr static bool dummy ## _ ## X = store(STR_HELPER(_##X), offsetOf(&C::_##X) );


#define VARREF(X, Y)    \
    protected: Y &_ ## X; \
    public: Y & X( ) { return _ ## X; }

#define RVAR(X, Y)    \
    protected: Y _ ## X; \
    public: Y  X( ) { return _ ## X; }

#define RWVAR(X, Y)    \
    protected: Y _ ## X; \
    public: Y & X( ) { return _ ## X; }

#define NVAR(X, Y, Z) \
    protected: Y Z;       \
    public: Y & X( ) { return Z; }
#endif



#ifndef FOREACH
#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); it++)
#endif


#define X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE(r, data, elem)    \
    case elem : return BOOST_PP_STRINGIZE(elem);

#define DEFINE_ENUM_WITH_STRING_CONVERSIONS(name, enumerators)                \
    enum name {                                                               \
    BOOST_PP_SEQ_ENUM(enumerators)                                        \
    };                                                                        \
    \
    inline const char* ToString(name v)                                       \
{                                                                         \
    switch (v)                                                            \
{                                                                     \
    BOOST_PP_SEQ_FOR_EACH(                                            \
    X_DEFINE_ENUM_WITH_STRING_CONVERSIONS_TOSTRING_CASE,          \
    name,                                                         \
    enumerators                                                   \
    )                                                                 \
    default: return "[Unknown " BOOST_PP_STRINGIZE(name) "]";         \
    }                                                                     \
    }




#ifdef WIN32
#define strncasecmp(x, y, z)    _strnicmp(x, y, z)
#else
#define _strnicmp strncasecmp
#define S_OK 0
#define E_FAIL -1
inline void DebugBreak () { assert(0); }
#endif

#endif

