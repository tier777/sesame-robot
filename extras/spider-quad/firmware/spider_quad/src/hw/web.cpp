#include "web.h"

#include <WebServer.h>
#include <WiFi.h>

#include "web_page.h"

namespace spider {

namespace {

WebServer server(80);
WebCtx ctx;

// Whole-string integer parse; rejects empty strings, junk and out-of-range values.
bool parseInt(const String &s, int lo, int hi, int *out) {
  if (s.isEmpty() || s.length() > 6) return false;
  int i = (s[0] == '-') ? 1 : 0;
  if (i == (int)s.length()) return false;
  for (; i < (int)s.length(); ++i)
    if (s[i] < '0' || s[i] > '9') return false;
  int v = s.toInt();
  if (v < lo || v > hi) return false;
  *out = v;
  return true;
}

void sendCal() {
  String j = "{\"trim\":[";
  for (int i = 0; i < kJoints; ++i) j += String(i ? "," : "") + ctx.cal[i].trim;
  j += "],\"dir\":[";
  for (int i = 0; i < kJoints; ++i) j += String(i ? "," : "") + ctx.cal[i].dir;
  j += "],\"cycle\":" + String(ctx.motion->cycleMs()) + "}";
  server.send(200, "application/json", j);
}

void bad(const char *why) { server.send(400, "text/plain", why); }

void handleCmd() {
  const String c = server.arg("c");
  uint32_t now = millis();
  if (c == "walk") {
    const String d = server.arg("d");
    Dir dir;
    if (d == "fwd") dir = Dir::Forward;
    else if (d == "back") dir = Dir::Backward;
    else if (d == "left") dir = Dir::TurnLeft;
    else if (d == "right") dir = Dir::TurnRight;
    else return bad("d: fwd|back|left|right");
    ctx.motion->command(Cmd::Walk, now, dir);
  } else if (c == "stand") ctx.motion->command(Cmd::Stand, now);
  else if (c == "lie") ctx.motion->command(Cmd::Lie, now);
  else if (c == "sit") ctx.motion->command(Cmd::Sit, now);
  else if (c == "hello") ctx.motion->command(Cmd::Hello, now);
  else if (c == "dance") ctx.motion->command(Cmd::Dance, now);
  else if (c == "stop") ctx.motion->command(Cmd::Stop, now);
  else if (c == "center") ctx.motion->command(Cmd::Center, now);
  else return bad("unknown command");
  server.send(200, "text/plain", "ok");
}

void handleTrim() {
  int j, v;
  if (!parseInt(server.arg("j"), 0, kJoints - 1, &j)) return bad("j: 0..7");
  if (!parseInt(server.arg("v"), -kTrimLimit, kTrimLimit, &v)) return bad("v: -30..30");
  ctx.cal[j].trim = clampTrim(v);
  sendCal();
}

void handleDir() {
  int j;
  if (!parseInt(server.arg("j"), 0, kJoints - 1, &j)) return bad("j: 0..7");
  ctx.cal[j].dir = int8_t(-ctx.cal[j].dir);
  sendCal();
}

void handleSpeed() {
  int ms;
  if (!parseInt(server.arg("ms"), 0, 60000, &ms)) return bad("ms: number");
  ctx.motion->setCycleMs(uint16_t(ms));
  ctx.store->saveCycle(ctx.motion->cycleMs());
  sendCal();
}

}  // namespace

void webBegin(const WebCtx &c, const char *ssid, const char *pass) {
  ctx = c;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pass);
  server.on("/", [] { server.send_P(200, "text/html; charset=utf-8", kPage); });
  server.on("/cmd", handleCmd);
  server.on("/cal", sendCal);
  server.on("/trim", handleTrim);
  server.on("/dir", handleDir);
  server.on("/speed", handleSpeed);
  server.on("/save", [] {
    ctx.store->save(ctx.cal);
    server.send(200, "text/plain", "saved");
  });
  server.onNotFound([] { server.send(404, "text/plain", "not found"); });
  server.begin();
}

void webLoop() { server.handleClient(); }

}  // namespace spider
