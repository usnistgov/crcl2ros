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

#include <iostream>
#include <string>

#include <boost/regex.hpp>

#include "crclapp/CrclSocketServer.h"

using namespace crcl;

bool CBufferHandler::_bTrace;

static std::string trim (std::string source, std::string delims = " \t\r\n")
{
    std::string result = source.erase(source.find_last_not_of(delims) + 1);
    return result.erase(0, result.find_first_not_of(delims));
}

////////////////////////////////////////////////////////////////////////////////
///  CBufferHandler
////////////////////////////////////////////////////////////////////////////////

CBufferHandler::CBufferHandler(CSocketSession * pSession)
{
    this->pSession=pSession;
}
CBufferHandler::~CBufferHandler()
{


}

////////////////////////////////////////////////////////////////////////////////
void CBufferHandler::AppendBuffer(std::string read)
{
    if (_next.size() > 0) {
        _current.append(_next);
        _next.clear();
    }
    size_t oldsize = _current.size();
    _current.append(read);
    if (_endtag == NonsenseTag()) {
        _endtag = FindLeadingElement(_current);

        if (_endtag.empty()) {
            _endtag = NonsenseTag();
        }
    }
    BufferHandler(_endtag);
}

////////////////////////////////////////////////////////////////////////////////
void CBufferHandler::SaveMessage(std::string xmlmessage) {
    if (CBufferHandler::_bTrace)
    {
                ROS_DEBUG("==========================================================="
                "%s",
                        xmlmessage.c_str());
    }

    CSocketSession::InMessages().addMsgQueue(boost::make_tuple(xmlmessage, pSession));

    // quirky, can cause core dump in mutex
    //pSession->crcl2ros->wake();

}

////////////////////////////////////////////////////////////////////////////////
std::string CBufferHandler::FindLeadingElement(std::string xml)
{
    boost::match_results<std::string::const_iterator> matchResult;
    bool found;
    boost::regex e("<[A-Za-z0-9_]+");
    found = boost::regex_search(xml, matchResult, e);

    if (found) {
        std::string elem(matchResult[0]);
        elem.insert(1, 1, '/');
        elem = trim(elem);
        elem.append(">"); // not space
        return elem;
    }
    return NonsenseTag();
}

////////////////////////////////////////////////////////////////////////////////
void CBufferHandler::BufferHandler(std::string & endtag)
{    int nMaxConnections;
     int nConnections;

      std::size_t found;

       while ((found = _current.find(endtag)) != std::string::npos) {
           found = found + endtag.size();
           _next = _current.substr(found);
           _current = _current.substr(0, found);
           SaveMessage(_current);
           //_inmsgs.AddMsgQueue(boost::make_tuple(_current, this) );
           _current = _next; // MISSING? when messages are pumped out back to back
           _next.clear();
           endtag = FindLeadingElement(_current);
       }
}
////////////////////////////////////////////////////////////////////////////////
///  CCrclSession
////////////////////////////////////////////////////////////////////////////////
int CSocketSession::dbgCrclXML;

//CMessageQueueThread *  CCrclSession::_inmsgs= NULL;  // queue with thread notify
CMessageQueue *  CSocketSession::_inmsgs= NULL;  // queue with thread notify

////////////////////////////////////////////////////////////////////////////////
CSocketSession::CSocketSession(double cycletime,
                           std::string name,
                           int port)   :
    RCS::Thread(cycletime, name)
{

    _remoteport=port;
    RCS::Thread::cycleTime()=cycletime;
    bufHandler = std::shared_ptr<CBufferHandler>( new CBufferHandler(this));
    mSocketServer=new SocketServer(port,(cycletime*1000),
                             (std::shared_ptr<IBufferHandler>) bufHandler.get());


}


////////////////////////////////////////////////////////////////////////////////
void CSocketSession::init()
{


}

////////////////////////////////////////////////////////////////////////////////
int CSocketSession::action()
{
    MultiClientSocket *client = mSocketServer->connectToClients();
    //          if (client != NULL)
    //            sendInitialData(client);

    mSocketServer->readFromClients();


    return 1;
}
////////////////////////////////////////////////////////////////////////////////
void CSocketSession::SyncWrite(std::string str)
{
    mSocketServer->sendToClients(str.c_str());
}
////////////////////////////////////////////////////////////////////////////////
void CSocketSession::stop (bool bWait)
{
    // disconnect clients and delete socket
    delete this->mSocketServer;
}

////////////////////////////////////////////////////////////////////////////////
void CSocketSession::cleanup()
{

}
