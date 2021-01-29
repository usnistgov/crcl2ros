#ifndef SOCKETSERVER_H
#define SOCKETSERVER_H
/*
* Copyright (c) 2008, AMT – The Association For Manufacturing Technology (“AMT”)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the AMT nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* DISCLAIMER OF WARRANTY. ALL MTCONNECT MATERIALS AND SPECIFICATIONS PROVIDED
* BY AMT, MTCONNECT OR ANY PARTICIPANT TO YOU OR ANY PARTY ARE PROVIDED "AS IS"
* AND WITHOUT ANY WARRANTY OF ANY KIND. AMT, MTCONNECT, AND EACH OF THEIR
* RESPECTIVE MEMBERS, OFFICERS, DIRECTORS, AFFILIATES, SPONSORS, AND AGENTS
* (COLLECTIVELY, THE "AMT PARTIES") AND PARTICIPANTS MAKE NO REPRESENTATION OR
* WARRANTY OF ANY KIND WHATSOEVER RELATING TO THESE MATERIALS, INCLUDING, WITHOUT
* LIMITATION, ANY EXPRESS OR IMPLIED WARRANTY OF NONINFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.

* LIMITATION OF LIABILITY. IN NO EVENT SHALL AMT, MTCONNECT, ANY OTHER AMT
* PARTY, OR ANY PARTICIPANT BE LIABLE FOR THE COST OF PROCURING SUBSTITUTE GOODS
* OR SERVICES, LOST PROFITS, LOSS OF USE, LOSS OF DATA OR ANY INCIDENTAL,
* CONSEQUENTIAL, INDIRECT, SPECIAL OR PUNITIVE DAMAGES OR OTHER DIRECT DAMAGES,
* WHETHER UNDER CONTRACT, TORT, WARRANTY OR OTHERWISE, ARISING IN ANY WAY OUT OF
* THIS AGREEMENT, USE OR INABILITY TO USE MTCONNECT MATERIALS, WHETHER OR NOT
* SUCH PARTY HAD ADVANCE NOTICE OF THE POSSIBILITY OF SUCH DAMAGES.
*/

#include "internal.hpp"
#include <mutex>
#include <memory>

class IBufferHandler
{
public:
    /*!
     * \brief Appends a socket buffer. Add previous buffer is exists.
     * Override this virtual function to implement stream buffer handler.
     * \param read buffer of characters
     */
    virtual void AppendBuffer(std::string read);
};

class MultiClientSocket;

/* Some constants */
const int MAX_CLIENTS = 64;

/* A socket server abstraction */
class SocketServer
{
protected:
  SOCKET mSocket;
  MultiClientSocket *mClients[MAX_CLIENTS + 1];
  int mNumClients;
  int mPort;
  int mTimeout;
  static std::mutex socketmutex;
  std::shared_ptr<IBufferHandler> bufferHandler;

protected:
  // Assumes the mutex is already locked.
  void removeClientInternal(MultiClientSocket *aClient);

  // Locks the mutex.
  void removeClient(MultiClientSocket *aClient);

  MultiClientSocket *addClient(MultiClientSocket *aClient);
  unsigned int getTimestamp();
  unsigned int deltaTimestamp(unsigned int, unsigned int);

public:
  SocketServer(int aPort,
               int aHeartbeatFreq,
               std::shared_ptr<IBufferHandler> bufhandler=nullptr);
  ~SocketServer();

  // Returns the new client.
  MultiClientSocket *connectToClients(); /* Client factory */

  /* I/O methods */
  void readFromClients();         /* discard data on read side of
                                                  sockets */
  void sendToClients(const char *aString);
  void sendToClient(MultiClientSocket *aClient, const char *aString);

  /* Getters */
  int numClients() { return mNumClients; }
  bool hasClients() { return mNumClients > 0; }

};

#endif // SOCKETSERVER_H
