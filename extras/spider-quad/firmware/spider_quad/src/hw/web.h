// Wi-Fi access point and the control page (http://192.168.4.1).
#pragma once
#include "../core/motion.h"
#include "store.h"

namespace spider {

struct WebCtx {
  Motion *motion;
  JointCal *cal;  // kJoints entries, edited live by the page
  Store *store;
};

void webBegin(const WebCtx &ctx, const char *ssid, const char *pass);
void webLoop();

}  // namespace spider
