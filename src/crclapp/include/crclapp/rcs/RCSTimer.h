//

// RCSTimer.h
//

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
*/

#ifndef _RCSTIMER_H
#define _RCSTIMER_H

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string.h>
#include <thread>
#include <sys/time.h>

#include <condition_variable>

namespace RCS
{
class Timer;
/* prototype for signal hander function */
typedef int (*RCS_TIMERFUNC)(void *_arg);
/* sleeps # of seconds, to clock tick resolution */

// this will be a fraction of seconds
// Use: RCS::esleep(10.0);  // sleep ten seconds

template<class Rep, class Period>
double ToNanoseconds (std::chrono::duration<Rep, Period> d)
{
    return static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(d).count( ) );
}
template<class Rep, class Period>
double ToSeconds (std::chrono::duration<Rep, Period> d)
{
    return static_cast<double>(
                std::chrono::duration_cast<std::chrono::seconds>(d).count( ) );
}
typedef std::chrono::high_resolution_clock::time_point RCS_Time;
/**
* \brief Timer is a general-purpose timer.
* The Timer is a general-purpose timer, which can be used for waiting until a
* synchronous time tick, sleep for any period at all, or to
* obtain a time in system clock ticks from creation of the timer.
*/
class Timer
{
public:

    /*!
        * \brief timeout is wait interval, rounded up to clock tick resolution;
        * function is external time base, if provided.
        * \param  t timeout period.
        */
    Timer(double t, RCS_TIMERFUNC _function = (RCS_TIMERFUNC) 0)
    {
        zeroTimer( );

        if ( t < _clkTckVal )
        {
            /* bump interval up to minimum system clock tick */
            this->_timeout = _clkTckVal;
        }
        else
        {
            this->_timeout = t;
        }
        _function = _function;
        _idle     = 0.0;                                /* set accumulated idle time to 0.0 */
        _counts   = 0;                                  /* set accumulated waits to 0 */

        _lastTime  = etime( );                         /* initialize start time and last time called	to
                             current time since epoch */
        _startTime = etime( );                         /* set creation time to now */
        _timeSinceRealSleep
                = _startTime;                              // boost::chrono::time_point_cast<boost::chrono::microseconds>(start_time);
        // //  start_time;
        _bSuspend = false;
    }

    /*!
        * \brief return last sleep number of seconds to slept.
        * \return  last seconds (or fractions) last slept. -199.99 if unused.
        */
    static double & lastEsleepSecondsToSleep ( )
    {
        static double last_esleep_seconds_to_sleep = -199.99;

        return last_esleep_seconds_to_sleep;
    }

    static int & etimeDisabled ( )
    {
        static int etime_disabled = 0;

        return etime_disabled;
    }

    static double & etimeDisableTime ( )
    {
        static double etime_disable_time = 0;

        return etime_disable_time;
    }

    /*!
        * \brief sleep number of seconds to sleep.
        * \param  seconds (or fractions) to sleep. Must be positive.
        */
    void esleep (double seconds_to_sleep)
    {
        Timer::lastEsleepSecondsToSleep( ) = seconds_to_sleep;

        if ( seconds_to_sleep <= 0.0 )
        {
            return;
        }

        // convert seconds to nanoseconds
        double                   nanosec = seconds_to_sleep * 1E09;
        std::chrono::nanoseconds nanosleep((long long) nanosec);
        std::this_thread::sleep_for(nanosleep);
    }
    std::mutex & mymutex()
    {
        static std::mutex mytimermutex;
        return mytimermutex;
    }
    std::condition_variable mycond;
    bool flag;
    bool esleepWBreak(double seconds_to_sleep)
    {
        flag=false;
        std::unique_lock<std::mutex> lock(mymutex());
        // convert seconds to nanoseconds
        double  nanosec = seconds_to_sleep * 1E09;
        std::chrono::nanoseconds nanosleep((long long) nanosec);

//        auto start = std::chrono::steady_clock::now();
//        std::cout << "Started sleep...";

        mycond.wait_for( lock,
                         nanosleep,
                         [&]() { return flag; } );
//        std::cout
//            << "Ended sleep after "
//            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//            << "ms.\n";
        return flag;
    }

    void wake()
    {
        std::lock_guard<std::mutex> lock(mymutex());
        flag=true;
        mycond.notify_one();
    }

    /*!
        * \brief number of seconds from some epoch, to clock tick resolution.
        * \return  high_resolution_clock now
        */
    static std::chrono::high_resolution_clock::time_point etime ( )
    {
        return std::chrono::high_resolution_clock::now( );
    }

    /*!
        * \brief number of clock ticks per second using high resolution timer.
        * \return number of ticks per second.
        */
    double clkTck ( )
    {
        return ( (double) std::chrono::high_resolution_clock::period::num )
                / ( (double) std::chrono::high_resolution_clock::period::den );
    }

    /*!
        * \brief wait on synch; returns # of cycles missed.
        * \return # of cycles missed.
        */
    int wait ( )
    {
        // boost::chrono::high_resolution_clock::time_point interval;		/* interval
        // between this and last wakeup */
        double numcycles;                              /* interval, in units of timeout */
        int    missed    = 0;                          /* cycles missed */
        double remaining = 0.0;                        /* time remaining until timeout */

        std::chrono::high_resolution_clock::time_point
                time_in;                                   /* time wait() was entered */
        std::chrono::high_resolution_clock::time_point
                time_done;                                 /* time user function finished */

        if ( _bSuspend )
        {
            std::unique_lock<std::mutex> aLock(_condMutex, std::try_to_lock);

            // std::mutex::scoped_lock aLock(condMutex);
            _cWakeup.wait(aLock);
        }

        /* first call the user timing function, if any */
        if ( _function != 0 )
        {
            /* set time in */
            time_in = etime( );

            if ( ( *_function )(0) == -1 )
            {
                return -1;                             /* fatal error in timing function */
            }
            time_done = etime( );
        }
        else
        {
            /* set time in, time done not used */
            time_in = etime( );
        }

        /* calculate the interval-- for user timing functions, this is how
            long between this wakeup and the last wakeup.  For internal timers,
            this is how long we need to sleep to make it to the next interval
            on time. */
        double interval = ToNanoseconds(time_in - _lastTime) / 1E9;
        numcycles = interval / _timeout;

        /* synchronize and set last_time correctly; update idle time */
        _counts++;

        if ( _function != 0 )
        {
            _lastTime = time_done;
        }
        _idle     += _timeout > interval ? _timeout - interval : 0;
        _busy     += interval;
        missed    = (int) numcycles;
        remaining = _timeout - interval;
        //esleep(remaining);
        esleepWBreak(remaining);
        _lastTime = etime( );
        return missed;
    }

    /*!
        * \brief Returns % loading on timer, 0.0 means all waits, 1.0 means no time in
        * wait. This is average load.
        * \return double or -1 of time spent busy.
        */
    double load ( )
    {
        if ( _counts * _timeout > 1e-9 )
        {
            return _busy / ( _counts * _timeout );
        }
        return -1.0;
    }

    /*!
        * \brief Compute free time over all cycles.
        */
    double free ( )
    {
        if ( _counts * _timeout > 1e-9 )
        {
            return _idle / ( _counts * _timeout );
        }
        return -1.0;
    }

    /*!
        * \brief Synchronize the timing service.
        * Initialize start time and last time called to current time since epoch.
        */
    void sync ( )
    {
        _lastTime = etime( );                          /* initialize start time and last time called to
                            current time since epoch */
    }

    /*!
        * \brief Suspend the timing.
        */
    void suspend ( ) { _bSuspend = true; }

    /*!
        * \brief Resume the timing. Wakeup timer with boost conditional notify.
        */
    void resume ( )
    {
        _bSuspend = false;
        _cWakeup.notify_all( );
    }

    std::string strNowDatetime ( )
    {
        using namespace std;
        using namespace std::chrono;
        system_clock::time_point tp = std::chrono::high_resolution_clock::now( );

        auto         ttime_t = system_clock::to_time_t(tp);
        auto         tp_sec  = system_clock::from_time_t(ttime_t);
        milliseconds ms      = duration_cast<milliseconds>(tp - tp_sec);

        std::tm *ttm = localtime(&ttime_t);

        char date_time_format[] = "%Y/%m/%d-%H:%M:%S";

        char time_str[] = "yyyy/mm/dd HH:MM:SS.fff";

        strftime(time_str, strlen(time_str), date_time_format, ttm);

        string result(time_str);
        result.append(".");
        result.append(to_string(ms.count( )));
        return result;
    }

    std::string strNowTime ( )
    {
        using namespace std;
        using namespace std::chrono;
        system_clock::time_point tp = std::chrono::high_resolution_clock::now( );

        auto         ttime_t = system_clock::to_time_t(tp);
        auto         tp_sec  = system_clock::from_time_t(ttime_t);
        milliseconds ms      = duration_cast<milliseconds>(tp - tp_sec);

        std::tm *ttm = localtime(&ttime_t);

        char date_time_format[] = "%H:%M:%S";

        char time_str[] = "HH:MM:SS.fff";

        strftime(time_str, strlen(time_str), date_time_format, ttm);

        string result(time_str);
        result.append(".");
        result.append(to_string(ms.count( )));
        return result;
    }

    static void setnow (struct timeval *tv)
    {
        auto now       = std::chrono::system_clock::now( );
        auto millisecs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch( ));

        tv->tv_sec  = millisecs.count( ) / 1000;
        tv->tv_usec = ( millisecs.count( ) % 1000 ) * 1000;
    }

    double &busy() { return _busy; }
    int &counts() { return _counts; }
    Timer & operator = (const Timer & _timer)
    {
        ( *this ) = _timer;
        return *this;
    }

    void setTimeout (double t) { this->_timeout = t; }
    double & timeout () { return this->_timeout ; }

private:
    /*!
        * \brief zeroes out the class parameters.
        */
    void zeroTimer ( )
    {
        _function                = 0;
        _busy                    = 0.0;
        _idle                    = 0.0;                 /* set accumulated idle time to 0.0 */
        _counts                  = 0;                   /* set accumulated waits to 0 */
        _startTime              = etime( );            /* set creation time to now */
        _timeSinceRealSleep   = _startTime;
        _countsPerRealSleep   = 0;
        _countsSinceRealSleep = 0;
        _clkTckVal             = clkTck( );
        _timeout                 = _clkTckVal;
        _bSuspend               = false;
    }

    double _timeout;                                    /**< copy of timeout value */
    double _busy;                                       /**< accumulated busy time in seconds*/
    int _counts;                                        /**< accumulated wait cycles */

    RCS_TIMERFUNC _function;                            /**< copy of function */

    std::chrono::high_resolution_clock::time_point _startTime, _lastTime;
    std::chrono::high_resolution_clock::time_point _timeSinceRealSleep;
    std::condition_variable _cWakeup;                   /**<  conditional value to wakeup timer */
    std::mutex _condMutex;                              /**<  mutex on timer */

    bool _bSuspend;                                    /**< suspend timer wait flag */
    double _clkTckVal;                                /**< clock tick seconds*/
    double _idle;                                       /**< accumulated idle time in seconds*/
    int _countsSinceRealSleep;                       /**< integer value of counts since last sleep*/
    int _countsPerRealSleep;                         /**< integer value of counts per sleep*/
};
}
#endif
