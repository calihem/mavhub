#ifndef _CTRL_WIFIMETER_H_
#define _CTRL_WIFIMETER_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#ifdef HAVE_LIBGPSMM_H

#include <string>
#include "core/protocollayer.h"

namespace mavhub
{
  class Ctrl_Wifimeter : public AppLayer
  {
    public:
      /// Constructor
      Ctrl_Wifimeter(const std::map<std::string, std::string> args);
      virtual ~Ctrl_Wifimeter();
      /// mavhub protocolstack input handler
      virtual void handle_input(const mavlink_message_t &msg);
    protected:
      /// this thread's main method
      virtual void run();
    private:
      /// component id
      uint16_t component_id;
      
      // gps stuff
      double latitude;
      double longitude;
      double xdop;
      double ydop;
      int satUsed;
      int satVis;

      // baro stuff
      double altitude;

      // socket stuff
      int port;
      std::string source;
      double freq;
  };
}

#endif // HAVE_GPSMM_H
#endif // _CTRL_WIFIMETER_H_
