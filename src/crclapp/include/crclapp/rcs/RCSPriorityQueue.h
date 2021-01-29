
/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#ifndef RCSPRIORITYQUEUE_H
#define RCSPRIORITYQUEUE_H
#include <queue>
#include <deque>
#include <mutex>
#include <algorithm>

#define SCOPED_LOCK    std::lock_guard<std::mutex> guard(m)

namespace msgq = std;


namespace RCS
{



/**
 * \brief The CMessageQueue offers a mutexed front to a STL/std deque. The queue
 * is a LIFO data structure.
 *   Useful for safely sharing data between multiple threads.
 */
    template<typename T, typename Comparator>
    class CPriorityMessageQueue
    {
public:
        typedef std::deque<T>                      xml_message_queue;
        typedef typename std::deque<T>::iterator   xml_message_queue_iterator;

        CPriorityMessageQueue( ) { }

        /// \brief ClearMsgQueue clears all contents in message queue. T

        void clear ( )
        {
            SCOPED_LOCK;
            xmlMsgs.clear( );
        }

        T item(size_t i)
        {
            if ( !xmlMsgs.size( ) )
                throw std::runtime_error("Empty queue\n");
            if(i>=xmlMsgs.size( ) || i < 0)
                throw std::runtime_error("Empty queue\n");
            return xmlMsgs[i];
        }

        /// \brief SizeMsgQueue returns number of items in message queue.

        size_t size ( )
        {
            SCOPED_LOCK;
            return xmlMsgs.size( );
        }

        /*!
         * \brief PopFrontMsgQueue mutex pop of front item of message queue.
         * \return  T       returns front item from message queue.
         */
        T pop ( )
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
         * \brief peek mutex peeks at front item of message queue.
         * \return  T       returns front item from message queue.
         */
        T &peek ( )
        {
            SCOPED_LOCK;

            if ( !xmlMsgs.size( ) )
            {
                throw std::runtime_error("Empty queue\n");
            }
            //T msg = xml_msgs.front( );
            return xmlMsgs.front( );
        }

        /*!
         * \brief poke mutex replace front item of message queue.
         * \param t type T to use as replacement.
         */
        void poke (T t )
        {
            SCOPED_LOCK;

            if ( !xmlMsgs.size( ) )
            {
                throw std::runtime_error("Empty queue\n");
            }
            xmlMsgs.front( )=t;
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

        /*!
         * \brief AddMsgQueue mutex push to back an item onto message queue.
         * \param  T       item to place in back of message queue.
         */
        void addMsgQueue (T t)
        {
            SCOPED_LOCK;
            xmlMsgs.push_back(t);
            std::sort(xmlMsgs.begin(), xmlMsgs.end(), Comparator());
        }


protected:
        msgq::mutex m;
        xml_message_queue xmlMsgs;
    };


}
#endif // RCSPRIORITYQUEUE_H
