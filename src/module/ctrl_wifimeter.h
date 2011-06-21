#ifndef _CTRL_WIFIMETER_H_
#define _CTRL_WIFIMETER_H_

#include "core/protocollayer.h"

namespace mavhub
{
  class Ctrl_Wifimeter : public AppLayer
  {
		/// Constructor
    CtrlWifimeter(int pport, string ssource, double ffeq);
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
    string source;
    double freq;
  };
}
#endif
