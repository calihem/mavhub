#ifndef _CTRL_WIFIMETER_CPP_
#define _CTRL_WIFIMETER_CPP_

#ifdef HAVE_LIBGPSMM_H
#include "ctrl_wifimeter.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <map>
#include <iomanip>
#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/time.h>
#include <libgpsmm.h>

#include <mavlink.h>
#include "core/logger.h"

namespace mavhub
{

Ctrl_Wifimeter::Ctrl_Wifimeter(const std::map<std::string, std::string> args):
  AppLayer("ctrl_wifimeter"),
  component_id(1000)
{
  std::map<std::string, std::string>::const_iterator iter;

  iter = args.find("port");
  if( iter != args.end() ) {
    std::istringstream s(iter->second);
    s >> this->port;
  }
  iter = args.find("source");
  if( iter != args.end() ) {
    std::istringstream s(iter->second);
    s >> this->source;
  }
  iter = args.find("freq");
  if( iter != args.end() ) {
    std::istringstream s(iter->second);
    s >> this->freq;
  }
}

Ctrl_Wifimeter::~Ctrl_Wifimeter()
{
  // nothing
}

void Ctrl_Wifimeter::handle_input(const mavlink_message_t &msg)
{
  if(msg.msgid == MAVLINK_MSG_ID_HUCH_CTRL_HOVER_STATE)
	  this->altitude = mavlink_msg_huch_ctrl_hover_state_get_baro(&msg);
}

void Ctrl_Wifimeter::run()
{
  //--- Initialize Socket ---//  
  int sd, interval;
  struct sockaddr_in target, local;
  struct hostent *lp/*, *gethostbyname()*/;
  
  int enableBroadcast = 1;
  
  lp = gethostbyname(this->source.c_str());
  interval = (int) 1e6/this->freq;

  sd = socket (AF_INET,SOCK_DGRAM,0);
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  bcopy ( lp->h_addr, &(local.sin_addr.s_addr), lp->h_length);
  setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &enableBroadcast, sizeof(enableBroadcast));
 
  if(bind(sd, (struct sockaddr*)&local, sizeof(local)) != 0) 
  {
    printf("bind() failed!\n");
    return;
  }

  memset(&target, 0, sizeof(struct sockaddr_in));
  target.sin_family = AF_INET;
  target.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  target.sin_port = htons(this->port);

  //--- Initialize GPS ---//
  gpsmm gpsRec;
  gpsRec.open("localhost", "2947");

  gpsRec.stream(WATCH_ENABLE|WATCH_JSON);

  struct gps_data_t *data;
    
  bool noFix = true;

  //--- Wait for Fix ---//  
  while(noFix)
  {
    data = gpsRec.poll();
    if(data->status == STATUS_FIX)
      noFix = false;
    usleep(interval);
  }

  //--- Start Sending ---//
  while(true)
  {
    data = gpsRec.poll();

    this->latitude = data->fix.latitude;
    this->longitude = data->fix.longitude;
    this->xdop = data->dop.xdop;
    this->ydop = data->dop.ydop;
    this->satUsed = data->satellites_used;
    this->satVis = data->satellites_visible;

    std::stringstream out;
    out << std::setprecision(7) << this->latitude << " ";
    out << std::setprecision(7) << this->longitude << " ";
    out << std::setprecision(7) << this->xdop << " ";
    out << std::setprecision(7) << this->ydop << " ";
    out << this->satUsed << " ";
    out << this->satVis << " ";
    out << this->altitude;

    if(sendto(sd, out.str().c_str(), out.str().size() * sizeof(char), 0, (struct sockaddr*) &target, sizeof(target)) == -1)
    {
      printf("sendto() failed!\n");
      return;
    }
    usleep(interval);
  }
}

}
#endif // HAVE_LIBGPSMM_H
#endif // _CTRL_WIFIMETER_CPP_
