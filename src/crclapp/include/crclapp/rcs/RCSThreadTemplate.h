//
// RCSThreadTemplate.h
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
#ifndef _RCS_ThreadTemplate_H
#define _RCS_ThreadTemplate_H

#include "RCSTimer.h"
#include <boost/thread.hpp>
#include <functional>

#include <rcs/Core.h>

namespace RCS
{
/**
 * \brief Thread is an RCS ulapi equivalent for a timed thread.
 * Given a cycle time, the thread provides a wait function to sleep to exactly
 * the
 * amount of the thread cycle time. It keeps track of busy/idle time for
 * diagnostic
 * purposes.
 * <BR> Notes:
 * https://www.quantnet.com/threads/c-multithreading-in-boost.10028/
 */
    class Thread
    {
public:

        /**
         * @brief Thread Constructor of thread, that takes cycle time and name as input.
         * @param cycletime cycle time of the thread in seconds
         * @param name string defining thread name
         */
        Thread(double cycletime, std::string name) : _timer(cycletime)
        {
            _bThread    = true;
            _cycletime  = cycletime;
            _debugLevel = 99;
            _bDone      = false;
            _name=name;
             Threads( ).push_back(this);
        }

        /*!
         * \brief Destructor of thread, makes sure thread has stopped.
         */
        ~Thread( ) { stop( ); }

        /*!
         * \brief ThreadGroup is a static definition of boost thread group.
         */
        static boost::thread_group & ThreadGroup ( )
        {
            static boost::thread_group _group;

            return _group;
        }

        /*!
         * \brief Threads is a static definition of all the threads that have been
         * created.
         */
        static std::vector<Thread *> & Threads ( )
        {
            static std::vector<Thread *> _group;

            return _group;
        }

        /*!
         * \brief Name returns name of thread.
         */
        std::string & name ( ) { return _name; }

        /*!
         * \brief Uses boost thread join routine.
         */
        void join ( ) { return m_thread.join( ); }

        /*!
         * \brief Init function called before Action() loop.
         */
        virtual void init ( ) { }

        /*!
         * \brief Cleanup function called after Action() loop done.
         */
        virtual void cleanup ( ) { }
        /**
         * @brief Preprocess called before every action cycle
         */
        virtual void preProcess ( ) { }

        /**
         * @brief Postprocess called after every action cycle
         */
        virtual void postProcess ( ) { }


        /**
         * @brief Action thread override function called every cycle
         * @return 0 stop thread, or else continue
         */
        virtual int action ( )
        {
            return -1;
        }

        /*!
         * \brief Start starts the thread which call init(), and then does action()
         * loop.
         */
        void start ( )
        {

            init( );
            _bThread = true;
            // m_thread = boost::thread(boost::bind(&Thread::Cycle, this));
            ThreadGroup( ).create_thread(boost::bind(&Thread::cycle, this));
        }

        /*!
         * \brief Static StopAll which stops all the threads created in the boost
         * thread group.
         */
        static void stopAll (bool bWait=false )
        {
            for ( int i = 0; i < Threads( ).size( ); i++ )
            {
                std::cout << "Stop Thread = " << Threads( ).at(i)->name( ).c_str( )
                          << std::endl;
                Threads( ).at(i)->stop( bWait);
            }

            if(!bWait)
                ThreadGroup( ).join_all( );
        }

        static void suspendAll ( )
        {
            for ( int i = 0; i < Threads( ).size( ); i++ )
            {
                std::cout << "Suspend Thread = " << Threads( ).at(i)->name( ).c_str( )
                          << std::endl;
                Threads( ).at(i)->suspend( );
            }
        }

        static void resumeAll ( )
        {
            for ( int i = 0; i < Threads( ).size( ); i++ )
            {
                std::cout << "Suspend Thread = " << Threads( ).at(i)->name( ).c_str( )
                          << std::endl;
                Threads( ).at(i)->resume( );
            }
        }
        static void initAll ( )
        {
            for ( int i = 0; i < Threads( ).size( ); i++ )
            {
                std::cout << "Init Thread = " << Threads( ).at(i)->name( ).c_str( )
                          << std::endl;
                Threads( ).at(i)->init( );
            }
        }
        /*!
         * \brief Stop stops the thread loop.
         * \param bWait indicates whether to wait until thread has finished.
         */
        void stop (bool bWait = false)
        {
            // wait until done
            while (bWait && !_bDone)
            {
                std::cout << "Thread " << this->name() << " waiting to finish\n";
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
            _bThread = false;
            _bDone = false;
        }

        /*!
         * \brief Suspend stops the thread loop until restarted with Resume().
         */
        void suspend ( ) { _timer.suspend( ); }

        /*!
         * \brief Resume resume execution of the thread loop stopped with Suspend().
         */
        void resume ( ) { _timer.resume( ); }

        /*!
         * \brief cpuLoad returns the load of the thread cycle.
         */
        double cpuLoad ( ) { return _timer.load( ); }

        /*!
         * \brief Load returns a string characterizing timing loads of all threads.
         */
        static std::string cpuLoads ( )
        {
            std::stringstream ss;
            ss << "Timing Load Threads\n" ;
            for ( int i = 0; i < Threads( ).size( ); i++ )
            {
                ss << "\t" << Threads( ).at(i)->name( ).c_str( ) << "\n" ;
                ss << "\t\tcycletime=" << Threads( ).at(i)->cycleTime() << "\n";
                ss << "\t\tload=" << Threads( ).at(i)->_timer.load( ) << "\n";
                ss << "\t\tbusy=" << Threads( ).at(i)->_timer.busy() << "\n";
                ss << "\t\tcounts=" << Threads( ).at(i)->_timer.counts()<< "\n";
                ss << "\t\ttimeout=" <<Threads( ).at(i)->_timer.timeout()<< "\n";
            }
            return ss.str();
        }


        /*!
         * \brief CycleTime returns the cycle time of the thread cycle in seconds.
         * \return double       returns cycle time of thread in seconds.
         */
        double & cycleTime ( ) { return _cycletime; }

        /*!
         * \brief DebugLevel returns the debugging level variable reference of the thread.
         * \return int       returns debug dlvel of thread.
         */
        int & debugLevel ( ) { return _debugLevel; }

        /*!
         * \brief Cycle is the thread main function. It calls init, action, and
         * cleanup.
         * After each cycle waits exactly amount given by cycle time.
         */
        void cycle ( )
        {
            _bDone = false;
             _timer.sync( );

            try
            {
                while ( _bThread )
                {
                    // Handle pre process routines
                    for(size_t i=0; i< _preprocess.size(); i++)
                        _preprocess[i]();

                    // Execute main loop, quit if false return
                    _bThread = (bool) action( );

                    // Handle post process routines
                    for(size_t i=0; i< _postprocess.size(); i++)
                        _postprocess[i]();

                    // wait till cycle time expired
                    _timer.wait( );                        // is this polling waiting or sleeping/yielding?
                }
            }
            catch ( ... )
            {
                std::cout << "Unhandled exception - " << name( ).c_str( ) << std::endl;
            }
            cleanup( );
            _bDone = true;
        }

        static void cycleAll ( )
        {
            Thread * thrd;
            try
            {
                for ( int i = 0; i < Threads( ).size( ); i++ )
                {

                    thrd =  Threads( ).at(i);
                    if(thrd->_bDone)
                        continue;

                    // Handle pre process routines
                    for(size_t i=0; i< thrd->_preprocess.size(); i++)
                        thrd->_preprocess[i]();

                    // Execute main loop, quit if false return
                    thrd->_bDone = (bool) thrd->action( );
                    thrd->_bDone =!thrd->_bDone;
                    // Handle post process routines
                    for(size_t i=0; i< thrd->_postprocess.size(); i++)
                        thrd->_postprocess[i]();

                    // wait till cycle time expired  no timer as sequention
                    //_timer.wait( );                        // is this polling waiting or sleeping/yielding?
                }
            }
            catch ( ... )
            {
                std::cerr << "Unhandled thread exception - " << thrd->name( ).c_str( ) << std::endl;
            }
        }
        void wake()
        {
            _timer.wake();
        }

        void addPreprocessFcn(std::function<void()> fcn)
        {
            _preprocess.push_back(fcn);
        }
        void addPostprocessFcn(std::function<void()> fcn)
        {
            _postprocess.push_back(fcn);
        }

        bool isDone() { return  _bDone; }
protected:
        std::vector<std::function<void()> > _preprocess;
        std::vector<std::function<void()> > _postprocess;
        std::string _name;                                 /**< name of thread */
        double _cycletime;                                 /**< cycletime of thread in seconds */
        int _debugLevel;                                   /**< debug level of thread */
        bool _bThread;                                     /**< boolean loop thread */
        bool _bDone;                                       /**< boolean indicating whether thread has finished */
        Timer  _timer;                                     /**< RCS timer for coordinating wait and duration of thread */
        boost::thread m_thread;                            /**< boost thread*/
    };
}
#endif
