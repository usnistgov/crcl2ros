
//
// RCSMsgQueue.h
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

#ifndef _RCS_QUEUE_H_
#define _RCS_QUEUE_H_
#include <deque>

// https://geidav.wordpress.com/2014/01/09/mutex-lock-guards-in-c11/
#include <mutex>

// std::mutex _mutex;
// Microsoft specific push/pop macros
#pragma push_macro("SCOPED_LOCK")
#undef SCOPED_LOCK
#define SCOPED_LOCK    std::lock_guard<std::mutex> guard(m)

namespace msgq = std;


namespace RCS
{
// * The CMessageQueue offers a thread safe message queue for buffering template
// types.

/**
 * \brief The CMessageQueue offers a mutexed front end to a STL/std deque. The queue
 * is a LIFO data structure.
 *   Useful for safely sharing data between multiple threads.
 */
template<typename T>
class CMessageQueue
{
public:
    typedef std::deque<T>                      xmlMessageQueue;
    typedef typename std::deque<T>::iterator   xmlMessageQueueIterator;

    /**
         * @brief CMessageQueue empty constructor.
         */
    CMessageQueue( ) { }

    /// \brief ClearMsgQueue clears all contents in message queue. T
    void clearMsgQueue ( )
    {
        SCOPED_LOCK;
        xmlMsgs.clear( );
    }

    /// \brief SizeMsgQueue returns number of items in message queue.
    size_t sizeMsgQueue ( )
    {
        SCOPED_LOCK;
        return xmlMsgs.size( );
    }

    /*!
         * \brief PopFrontMsgQueue mutex pop of front item of message queue.
         * \return  T       returns front item from message queue.
         */
    T popFrontMsgQueue ( )
    {
        SCOPED_LOCK;

        if ( !xmlMsgs.size( ) )
        {
            throw std::runtime_error("Empty queue\n");
        }
        T msg = xmlMsgs.front( );
        xmlMsgs.pop_front( );
        return msg;
    }

    /*!
         * \brief PeekFrontMsgQueue mutex peeks at front item of message queue.
         * \return  T       returns front item from message queue.
         */
    T peekFrontMsgQueue ( )
    {
        SCOPED_LOCK;

        if ( !xmlMsgs.size( ) )
        {
            throw std::runtime_error("Empty queue\n");
        }
        T msg = xmlMsgs.front( );
        return msg;
    }

    /*!
         * \brief BackMsgQueue mutex retrieves back item of message queue.
         * Does not pop queue.
         * \return  T       returns back item from message queue.
         */
    T backMsgQueue ( )
    {
        SCOPED_LOCK;

        if ( !xmlMsgs.size( ) )
        {
            throw std::runtime_error("Empty queue\n");
        }
        return xmlMsgs.back( );
    }


    /**
         * @brief popBackMsgQueue pops back item of message queue.
         * @return back item from queue
         */
    T popBackMsgQueue()
    {
        SCOPED_LOCK;

        if ( !xmlMsgs.size( ) )
        {
            throw std::runtime_error("Empty queue\n");
        }
        T msg = xmlMsgs.back( );
        xmlMsgs.erase( xmlMsgs.end()-1);
        return msg;

    }

    /*!
         * \brief AddMsgQueue muteendx push to back an item onto message queue.
         * \param  T       item to place in back of message queue.
         */
    void addMsgQueue (T t)
    {
        SCOPED_LOCK;
        xmlMsgs.push_back(t);
    }

    /*!
         * \brief AddMsgQueue mutex push to back an item onto message queue.
         * \param  T       item to place in back of message queue.
         */
    void addBackMsgQueue (T t)
    {
        SCOPED_LOCK;
        xmlMsgs.push_back(t);
    }

    /*!
         * \brief AddMsgQueue mutex push to front an item onto message queue.
         * \param  T       item to place in front of message queue.
         */
    void addFrontMsgQueue (T t)
    {
        SCOPED_LOCK;
        xmlMsgs.insert(xmlMsgs.begin( ), t);
    }

    /*!
         * \brief InsertFrontMsgQueue mutex push to front an item onto message queue.
         * \param  T       item to place in front of message queue.
         */
    void insertFrontMsgQueue (T t)
    {
        SCOPED_LOCK;

        xmlMsgs.insert(xmlMsgs.begin( ), t);
    }

    /**
         * @brief end retrieve end iterator of message queue
         * @return  xmlMessageQueueIterator
         */
    xmlMessageQueueIterator end ( ) { return xmlMsgs.end( ); }

    /**
         * @brief begin retrieve being iterator of message queue
         * @return  xmlMessageQueueIterator
         */
    xmlMessageQueueIterator begin ( ) { return xmlMsgs.begin( ); }
protected:
    msgq::mutex m;
    xmlMessageQueue xmlMsgs;
};
}
#pragma pop_macro("SCOPED_LOCK")

#endif
