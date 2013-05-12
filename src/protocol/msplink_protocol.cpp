#include "protocol.h"

#include <stdio.h>

namespace mavhub {

#ifdef HAVE_MSPLINK_H

  template <>
  int serialize<msp_message_t>(const msp_message_t &msg, uint8_t *buffer, const uint16_t length) {
    uint8_t checksum;
    // std::cout << "length:" << length << std::endl;

    // printf("length: %d\n", length);
    // printf("msg.sync: %d\n", msg.sync);
    // printf("msg.mw: %d\n", msg.mw);
    // printf("msg.dir: %d\n", msg.dir);
    // printf("msg.len: %d\n", msg.len);
    // printf("msg.type: %d\n", msg.type);

    // printf("length: %d\n", length);
    // int needed_size = 3+msg.len+1;
    // if(length < needed_size) return 0;

    // memcpy(buffer, &(msg.sync), 3);
    // memcpy(buffer+3, msg.data, msg.len);
    // memcpy(buffer+3+msg.len, &(msg.hash), 1);
    checksum = 0;
    buffer[0] = msg.sync;
    buffer[1] = msg.mw;
    buffer[2] = msg.dir;
    buffer[3] = msg.len;
    checksum ^= msg.len;
    buffer[4] = msg.type;
    checksum ^= msg.type;
    // insert payload here
    int i;
    for(i = 0; i < msg.len; ++i) {
      buffer[5+i] = msg.data[i];
      checksum ^= msg.data[i];
    }
    buffer[5+i] = checksum;

    // return needed_size;
    return(5+i+1);
  }

  template <>
  int parse_byte<msp_message_t>(const uint8_t input, uint16_t &index, msp_message_t &msg) {
    static msplink_status_t link_status;
    link_status.seq = index;

    int rc = msplink_parse_char(input, &msg, &link_status);
    index = link_status.seq;
    // printf("index: %d, rc: %d\n", index, rc);
    return rc;
  }

#endif // HAVE_MSPLINK_H

} //namespace mavhub
