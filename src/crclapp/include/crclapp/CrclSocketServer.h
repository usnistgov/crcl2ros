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
#ifndef CRCLSERVER_H
#define CRCLSERVER_H
// CrclSocketServer.h

#include <memory>

// ws header files
#include <rcs/RCSMsgQueue.h>
#include <rcs/RCSMsgQueueThread.h>
#include <rcs/Core.h>
#include <rcs/RCSThreadTemplate.h>

#include "MTCSOCKET/client.hpp"
#include "MTCSOCKET/socketserver.h"


namespace crcl{
class CSocketSession;
class CRosCmdMsgHandler;
class CRosCmdStatusHandler;


typedef boost::tuple<std::string, CSocketSession *> CrclMessage;
typedef RCS::CMessageQueue<CrclMessage> CCrclMessages;
typedef RCS::CMessageQueue<CrclMessage> CMessageQueue;

class CBufferHandler : public IBufferHandler
{
public:

    /**
     * @brief CBufferHandler constructor that takes a pointer to the parent session.
     */
    CBufferHandler(CSocketSession *);

    /**
      * @brief CBufferHandler destructor.
     */
    ~CBufferHandler();

    /**
     * @brief Session return pointer to session parent. not a smart pointer
     * as session should never be destroyed, as there really is only one session.
     * @return  pointer to CCrclSession, not a smart pointer.
     */
    CSocketSession * Session() { return pSession; }
    /*!
     * \brief Appends a socket buffer. Add previous buffer is exists.
     * \param data buffer of characters
     */
    void AppendBuffer(std::string data);

    /*!
     ** \brief Looks for matching end xml tag. If found, saves message into queue,
     * and restarts read process.
     * \param endtag is the ending tag, e.g., </ENDTAG to match against.  Includes backslash.
     */
    void BufferHandler(std::string & endtag);

    /*!
     * \brief Queues message onto message queue.
     * \param xmlmessage to queue onto this session message queue.
     */
    void SaveMessage(std::string xmlmessage);

    /*!
     * \brief Finds the leading XML tag to create matching end tag.
     * If none, return nonsense tag. Uses boost regex.
     * \oaram xml is the text to search for starting tag
     * \return end tag or nonsense tag if none. e.g., </TAG>
     */
    std::string FindLeadingElement(std::string xml);

    /*!
     *  \brief NonsenseTag to be used as dummy ending xml to test against.
     */
    static std::string NonsenseTag() {
        return "<XXXXX>";
    }
    CSocketSession * pSession;
    std::string _current; /**<  current string read from socket */
    std::string _next; /**<  leftover string after pulling out Crcl XML message */
    std::string _endtag; /**<  endtag to designate the end of Crcl XML message, found from beginning */
    static bool _bTrace;

};


class CSocketSession : public RCS::Thread
{
public:

    /**
     * @brief CCrclSession Constructor for each listener on the socket.
     * @param cycletime cycle time of thread in seconds
     * @param name of the thread
     */
    CSocketSession(double cycletime,
                 std::string name,
                 int port);
    SocketServer * mSocketServer;
    std::shared_ptr<CBufferHandler> bufHandler;


    std::string RemoteIP (){ return _remoteip; }
    unsigned short RemotePort (){ return _remoteport; }

    /**
     * @brief init handles socket setup.
     * create socket, allow reuse of port
     * ioctl setup for non-blocking socket
     * Create sockaddr_in structure for the server socket.
     * Using INADDR_ANY which means accept a connection on any of this host's IP addresses.
     * bind server socket (serverSock) to server address (serverAddr).
     * Necessary so that server can use a specific port
     * wait for a client by listening on socket.
     * Initialize the timeval struct to 3 milliseconds.
     * If no activity after 3 milliseconds skip
     */
    virtual void init();

    /**
     * @brief action handles checking for new connections, and reading data from all connections.
     * If no new data on connections, returns, otherwise calls BufferHandler to manage new data.
     * Copy the master fd_set over to the working fd_set.
     * Call select() and wait 5 minutes for it to complete.
     * If one or more descriptors are readable then need to determine which ones they are.
     * Check to see if this descriptor is ready
     * Check to see if this is the listening socket.
     * Accept all incoming connections that are queued up on the listening socket before we
     * loop back and call select again.
     * Receive all incoming data on this socket  before we loop back and call select again.
     * Receive data on this connection until the recv fails with EWOULDBLOCK.  If any other
     * failure occurs, we will close the    connection.
     *
     * @return 1 continue processing thread, 0 stop thread.
     */
    virtual int action();

    /**
     * @brief stop stop thread
     * @param bWait optional flag to wait until all underlying threads have ceased.
     */
    virtual void stop (bool bWait = false);

    /**
     * @brief SyncWrite writes the string to the CRCL socket
     * @param str  string to write
     */
    void SyncWrite(std::string str);

    /**
     * @brief cleanup overriden thread method to cleanup before quitting thread.
     */
    virtual void cleanup();

    /**
     * @brief InMessages queue of in messages from CRCL XML stream.
     * It is given as a pointer to a queue elsewhere in the system.
     * @return
     */
    static CMessageQueue & InMessages()
    {
        static CMessageQueue dummy;
        if(_inmsgs == nullptr)
        {
           ROS_FATAL("CSocketSession Message queue has not been assigned and is NULL");
           return dummy;
        }
        // should be copied?
        return *_inmsgs;
    }
    /**
     * @brief AssignMessageQueue assigns the output queue for incoming CRCL messages
     * @param msgq a CMessageQueue pointer type to assign
     */
    static void AssignMessageQueue(CMessageQueue * msgq)
    {
        _inmsgs=msgq;
    }


    static int dbgCrclXML; /**<  very verbose debug stream of CRCL XML input */
protected:
    static CMessageQueue * _inmsgs; /**<  queue of inbound crcl xml messages from device */

    char receivedStr[1000];

    std::string _remoteip; /**< ip, typically local host to read CRCL XML*/
    unsigned short _remoteport; /**<  port number to read CRCL XML stream on ip */
};

}
#endif // CRCLSERVER_H
