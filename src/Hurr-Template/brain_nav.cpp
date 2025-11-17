#include "Hurr-Template/brain_nav.h"
#include "robot-config.h"   // must declare Brain, motors, etc.
#include <cstring>
#include <cstdio>
#include <cstdint>

using namespace vex;

// ============================================================
//                  GLOBAL RUNTIME TABLES
// ============================================================

Binding      gBindings[32];
int          gBindingCount = 0;
MotorRuntime gRuntime[32];
int          gRuntimeCount = 0;

static MotorRuntime gSeedRuntime[32];
static int          gSeedCount = 0;
static bool         gRuntimeSeeded = false;

// ------------------------------------------------------------
//  Build alias → motor* bindings
//  NOTE: adjust this to match your actual robot-config motors.
// ------------------------------------------------------------
void buildBindings() {
  gBindingCount = 0;

  // These names assume you have these motors declared in robot-config.h
  // extern motor LeftFront, LeftMiddle, LeftBack, RightFront, RightMiddle, RightBack;
  // extern motor top_roller, intake;
  // Add/remove as needed.

  gBindings[gBindingCount++] = { "LeftFront",   &LeftFront   };
  gBindings[gBindingCount++] = { "LeftMiddle",  &LeftMiddle  };
  gBindings[gBindingCount++] = { "LeftBack",    &LeftBack    };

  gBindings[gBindingCount++] = { "RightFront",  &RightFront  };
  gBindings[gBindingCount++] = { "RightMiddle", &RightMiddle };
  gBindings[gBindingCount++] = { "RightBack",   &RightBack   };

  gBindings[gBindingCount++] = { "top_roller",  &top_roller  };
  gBindings[gBindingCount++] = { "intake",      &intake      };

  gRuntimeCount = gBindingCount;
}

// Helper: find runtime index by alias name
static int rtIndexByName_(const char* name) {
  for (int i = 0; i < gRuntimeCount && i < gBindingCount; ++i) {
    if (std::strcmp(gBindings[i].name, name) == 0) return i;
  }
  return -1;
}

// ------------------------------------------------------------
//  Seed runtime from your “build-time” config
//  You can adjust ports, gearings, reversed flags here.
// ------------------------------------------------------------
void seedRuntimeFromConfig() {
  if (gBindingCount == 0)
    buildBindings();

  // Default: all entries ratio6_1, not reversed, with ports guessed
  // from motor index() + 1. Then override specifics below.
  for (int i = 0; i < gBindingCount; ++i) {
    vex::motor* m = gBindings[i].ref;
    int p = (m ? (m->index() + 1) : 0);
    gRuntime[i] = { ratio6_1, false, p };
  }
  gRuntimeCount = gBindingCount;

  // ==== Hard-coded seed matching your earlier config ====
  auto setSeed = [](const char* name, int port, gearSetting gs, bool reversed) {
    int idx = rtIndexByName_(name);
    if (idx >= 0) {
      gRuntime[idx].port     = port;
      gRuntime[idx].gs       = gs;
      gRuntime[idx].reversed = reversed;
    }
  };

  // From your first robot-config snippet:
  // Left side
  setSeed("LeftFront",   4, ratio6_1, true);
  setSeed("LeftMiddle",  5, ratio6_1, false);
  setSeed("LeftBack",    6, ratio6_1, true);

  // Right side
  setSeed("RightFront",  1, ratio6_1, false);
  setSeed("RightMiddle", 2, ratio6_1, true);
  setSeed("RightBack",   3, ratio6_1, false);

  // Top roller and intake
  setSeed("top_roller",  9,  ratio6_1, true);
  setSeed("intake",      18, ratio6_1, true);

  // Copy to seed backup
  gSeedCount = gRuntimeCount;
  for (int i = 0; i < gRuntimeCount; ++i) {
    gSeedRuntime[i] = gRuntime[i];
  }
  gRuntimeSeeded = true;

  // Rebuild the actual motor objects to match these seeds
  for (int i = 0; i < gRuntimeCount && i < gBindingCount; ++i) {
    motor* m = gBindings[i].ref;
    if (!m) continue;
    m->stop();
    m->~motor();
    new (m) motor(gRuntime[i].port, gRuntime[i].gs, gRuntime[i].reversed);
  }
}

// ------------------------------------------------------------
//  Reset all runtime + motors back to seed config
// ------------------------------------------------------------
void reset_all_to_seed_() {
  if (!gRuntimeSeeded) return;

  gRuntimeCount = gSeedCount;
  for (int i = 0; i < gSeedCount; ++i) {
    gRuntime[i] = gSeedRuntime[i];
  }

  for (int i = 0; i < gRuntimeCount && i < gBindingCount; ++i) {
    motor* m = gBindings[i].ref;
    if (!m) continue;
    m->stop();
    m->~motor();
    new (m) motor(gRuntime[i].port, gRuntime[i].gs, gRuntime[i].reversed);
  }
}

// ------------------------------------------------------------
//  Runtime helpers used by BrainNav
// ------------------------------------------------------------
bool hotswap_reassign_by_alias(const char* aliasName, int newPort) {
  int idx = rtIndexByName_(aliasName);
  if (idx < 0) return false;
  if (newPort < 1 || newPort > 21) return false;

  // rebuild motor on new port
  motor* m = gBindings[idx].ref;
  if (!m) return false;
  m->stop();
  m->~motor();
  new (m) motor(newPort, gRuntime[idx].gs, gRuntime[idx].reversed);

  gRuntime[idx].port = newPort;
  return true;
}

bool hotswap_set_reverse_by_alias(const char* aliasName, bool newVal) {
  int idx = rtIndexByName_(aliasName);
  if (idx < 0) return false;

  gRuntime[idx].reversed = newVal;

  motor* m = gBindings[idx].ref;
  if (!m) return false;
  m->setReversed(newVal);
  return true;
}

bool hotswap_get_reverse_by_alias(const char* aliasName, bool& outVal) {
  int idx = rtIndexByName_(aliasName);
  if (idx < 0) return false;
  outVal = gRuntime[idx].reversed;
  return true;
}

int hotswap_get_port_by_alias(const char* aliasName) {
  int idx = rtIndexByName_(aliasName);
  if (idx < 0) return 0;
  return gRuntime[idx].port;
}

bool hotswap_get_gearing_by_alias(const char* aliasName, gearSetting& outGs) {
  int idx = rtIndexByName_(aliasName);
  if (idx < 0) return false;
  outGs = gRuntime[idx].gs;
  return true;
}

bool hotswap_is_active() {
  return (gRuntimeCount > 0);
}

// ============================================================
//                     BRAIN NAV IMPLEMENTATION
// ============================================================

static const int kW = 480;
static const int kH = 240;

BrainNav::BrainNav(MotorInfo* motors, int* countRef,
                   TempThresholds t, void (*rescanCb)())
  : motors_(motors),
    countRef_(countRef),
    tt_(t),
    rescan_(rescanCb) {}

// ------------------------------------------------------------
//  Touch helpers
// ------------------------------------------------------------
bool BrainNav::waitForRelease_() {
  while (Brain.Screen.pressing()) { task::sleep(5); }
  return true;
}

bool BrainNav::tappedRect_(int x, int y, int w, int h) {
  if (!Brain.Screen.pressing()) return false;
  int px = Brain.Screen.xPosition();
  int py = Brain.Screen.yPosition();
  return (px >= x && px <= x + w && py >= y && py <= y + h);
}

// ------------------------------------------------------------
//  Geometry helpers
// ------------------------------------------------------------
void BrainNav::gridGeom_(int& gx, int& gy, int& gw, int& gh,
                         int& cellW, int& cellH, int& rows) const {
  gx = margin_;
  gy = headerH_ + margin_;
  gw = kW - 2 * margin_;
  gh = kH - gy - margin_;

  int count = (countRef_ ? *countRef_ : 0);
  if (count < 0) count = 0;

  rows = (count + cols_ - 1) / cols_;
  if (rows < 1) rows = 1;

  cellW = (gw - (cols_ - 1) * gutter_) / cols_;
  cellH = (gh - (rows - 1) * gutter_) / rows;
}

int BrainNav::indexAt_(int px, int py) const {
  int gx, gy, gw, gh, cellW, cellH, rows;
  gridGeom_(gx, gy, gw, gh, cellW, cellH, rows);

  int count = (countRef_ ? *countRef_ : 0);
  if (count < 0) count = 0;

  for (int i = 0; i < count; ++i) {
    int r = i / cols_;
    int c = i % cols_;
    int x = gx + c * (cellW + gutter_);
    int y = gy + r * (cellH + gutter_);
    if (px >= x && px <= x + cellW && py >= y && py <= y + cellH) return i;
  }
  return -1;
}

// ------------------------------------------------------------
//  Color helpers
// ------------------------------------------------------------
vex::color BrainNav::tempToColor_(double c) const {
  if (c >= tt_.red)    return vex::color(220, 60, 60);
  if (c >= tt_.yellow) return vex::color(255, 185, 60);
  return vex::color(80, 200, 120);
}

vex::color BrainNav::darken_(const vex::color& /*c*/, double k) const {
  if (k < 0.0) k = 0.0;
  if (k > 1.0) k = 1.0;
  int r = static_cast<int>(k * 255);
  return vex::color(r, r, r);
}

vex::color BrainNav::penForTemp_(double c) const {
  if (c >= tt_.red)    return vex::color(255, 255, 255);
  if (c >= tt_.yellow) return vex::color(20, 20, 20);
  return vex::color(255, 255, 255);
}

// ------------------------------------------------------------
//  Name abbreviation
// ------------------------------------------------------------
const char* BrainNav::abbrev_(const char* name, char* buf, int bufsz) const {
  if (!name || !*name || bufsz < 2) {
    if (bufsz > 0) buf[0] = '\0';
    return buf;
  }
  int n = std::strlen(name);
  int out = 0;
  // first: uppercase letters
  for (int i = 0; i < n && out < bufsz - 1; ++i) {
    char ch = name[i];
    if (ch >= 'A' && ch <= 'Z') buf[out++] = ch;
  }
  if (out >= 2) { buf[out] = '\0'; return buf; }

  // else, initials
  out = 0;
  bool nw = true;
  for (int i = 0; i < n && out < bufsz - 1; ++i) {
    char ch = name[i];
    bool sep = (ch == ' ' || ch == '_' || ch == '-');
    if (nw && !sep) {
      buf[out++] = (ch >= 'a' && ch <= 'z') ? (ch - 'a' + 'A') : ch;
      nw = false;
    }
    if (sep) nw = true;
  }
  if (out >= 2) { buf[out] = '\0'; return buf; }

  // else first two non-seps
  out = 0;
  for (int i = 0; i < n && out < 2 && out < bufsz - 1; ++i) {
    char ch = name[i];
    if (ch == ' ' || ch == '_' || ch == '-') continue;
    buf[out++] = (ch >= 'a' && ch <= 'z') ? (ch - 'a' + 'A') : ch;
  }
  buf[out] = '\0';
  return buf;
}

// ------------------------------------------------------------
//  Port helper: check free
// ------------------------------------------------------------
bool BrainNav::isPortFree_(int port) const {
  // If any runtime entry uses this port, not free
  for (int i = 0; i < gRuntimeCount; ++i) {
    if (gRuntime[i].port == port) return false;
  }
  // Also probe physical motor to avoid overlapping with unknowns
  vex::motor probe(port);
  return !probe.installed();
}

const char* BrainNav::aliasForPortRT_(int port) const {
  for (int i = 0; i < gRuntimeCount && i < gBindingCount; ++i) {
    if (gRuntime[i].port == port) return gBindings[i].name;
  }
  return nullptr;
}

// ------------------------------------------------------------
//  Small widgets
// ------------------------------------------------------------
void BrainNav::drawPillToggle_(int x, int y, int w, int h, bool on, const char* label) {
  Brain.Screen.setPenColor(black);
  if (label) Brain.Screen.printAt(x - 100, y + h - 8, false, "%s", label);

  Brain.Screen.setFillColor(on ? vex::color(100, 200, 255) : vex::color(210,210,210));
  Brain.Screen.drawRectangle(x, y, w, h);

  int knobW = h - 4;
  int knobX = on ? (x + w - knobW - 2) : (x + 2);
  Brain.Screen.setFillColor(white);
  Brain.Screen.drawRectangle(knobX, y + 2, knobW, knobW);
}

// ------------------------------------------------------------
//  Screens
// ------------------------------------------------------------
void BrainNav::drawHome() {
  Brain.Screen.setFillColor(vex::color(245,245,245));
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(black);

  Brain.Screen.setFillColor(vex::color(230,230,230));
  Brain.Screen.drawRectangle(0, 0, kW, 36);
  Brain.Screen.printAt(12, 24, false, "%s", "Home");

  Brain.Screen.setFillColor(vex::color(210,210,210));
  Brain.Screen.drawRectangle(10, 10, 80, 24);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(20, 27, false, "%s", "< Back");

  int bw = 160, bh = 48;
  int bx = kW/2 - bw/2;
  int by = kH/2 - bh/2;

  Brain.Screen.setFillColor(vex::color(200,230,255));
  Brain.Screen.drawRectangle(bx, by, bw, bh);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(bx + 20, by + 30, false, "%s", "Motor Manager");
}

void BrainNav::drawMotors() {
  Brain.Screen.setFillColor(vex::color(235,235,235));
  Brain.Screen.setPenColor(black);
  Brain.Screen.drawRectangle(0, 0, kW, headerH_);
  Brain.Screen.printAt(12, 20, false, "%s", "Motors");

  Brain.Screen.setFillColor(vex::color(210,210,210));
  Brain.Screen.drawRectangle(10, 4, 80, headerH_ - 8);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(20, 20, false, "%s", "< Back");

  Brain.Screen.setFillColor(vex::color(255,220,220));
  Brain.Screen.drawRectangle(kW - 120, 4, 110, headerH_ - 8);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(kW - 112, 20, false, "%s", "Reset All");

  if (hotswap_is_active()) {
    Brain.Screen.setFillColor(vex::color(140, 210, 160));
    Brain.Screen.drawRectangle(kW - 280, 4, 150, headerH_ - 8);
    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(kW - 270, 20, false, "%s", "Runtime Remap: ON");
  }

  int gx, gy, gw, gh, cellW, cellH, rows;
  gridGeom_(gx, gy, gw, gh, cellW, cellH, rows);

  int count = (countRef_ ? *countRef_ : 0);
  if (count < 0) count = 0;

  for (int i = 0; i < count; ++i) {
    int r = i / cols_;
    int c = i % cols_;
    int x = gx + c * (cellW + gutter_);
    int y = gy + r * (cellH + gutter_);

    vex::motor* m = motors_[i].m;
    bool installed = m && m->installed();
    double tC = installed ? m->temperature(temperatureUnits::celsius) : 0.0;

    int port = 0;
    if (motors_[i].alias && *motors_[i].alias) {
      int rt = hotswap_get_port_by_alias(motors_[i].alias);
      if (rt > 0) port = rt;
    }
    if (port == 0 && installed) port = m->index() + 1;

    vex::color bg = tempToColor_(tC);
    Brain.Screen.setFillColor(bg);
    Brain.Screen.setPenColor(darken_(bg, 0.65));
    Brain.Screen.drawRectangle(x, y, cellW, cellH);

    Brain.Screen.setPenColor(penForTemp_(tC));

    char abuf[8];
    const char* n0 = motors_[i].name ? motors_[i].name : "Motor";
    const char* abbr = abbrev_(n0, abuf, sizeof(abuf));

    int cx = x + 8;
    int cy = y + 18;

    Brain.Screen.printAt(cx, cy, false, "%s", abbr);
    Brain.Screen.printAt(cx, cy + 16, false, "P:%d", port);
    if (installed)
      Brain.Screen.printAt(cx, cy + 32, false, "T:%.1fC", tC);
    else
      Brain.Screen.printAt(cx, cy + 32, false, "%s", "T:—");
  }
}

void BrainNav::drawMotorDetail() {
  Brain.Screen.setFillColor(vex::color(245,245,245));
  Brain.Screen.clearScreen();

  Brain.Screen.setFillColor(vex::color(235,235,235));
  Brain.Screen.setPenColor(black);
  Brain.Screen.drawRectangle(0, 0, kW, headerH_);
  Brain.Screen.printAt(12, 20, false, "%s", "Motor Details");

  Brain.Screen.setFillColor(vex::color(210,210,210));
  Brain.Screen.drawRectangle(10, 4, 80, headerH_ - 8);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(20, 20, false, "%s", "< Back");

  if (selected_ < 0) return;
  auto& mi = motors_[selected_];
  vex::motor* m = mi.m;

  bool ok   = m && m->installed();
  double tC = ok ? m->temperature(temperatureUnits::celsius) : 0.0;
  double rpm= ok ? m->velocity(velocityUnits::rpm) : 0.0;
  double watt=ok ? m->power(powerUnits::watt) : 0.0;

  int port = 0;
  if (mi.alias && *mi.alias) {
    int rt = hotswap_get_port_by_alias(mi.alias);
    if (rt > 0) port = rt;
  }
  if (port == 0 && ok) port = m->index() + 1;

  bool rev = false;
  gearSetting gs = ratio6_1;
  if (mi.alias && *mi.alias) {
    bool tmp=false; if (hotswap_get_reverse_by_alias(mi.alias, tmp)) rev = tmp;
    gearSetting gtmp=ratio6_1; if (hotswap_get_gearing_by_alias(mi.alias, gtmp)) gs = gtmp;
  }

  Brain.Screen.setPenColor(black);
  int x = 16, y = headerH_ + 24;
  Brain.Screen.printAt(x, y,      false, "Name: %s", mi.name ? mi.name : "(unnamed)");
  Brain.Screen.printAt(x, y + 20, false, "Port: %d", port);
  Brain.Screen.printAt(x, y + 40, false, "Temp: %.1f C", tC);
  Brain.Screen.printAt(x, y + 60, false, "RPM:  %.1f", rpm);
  Brain.Screen.printAt(x, y + 80, false, "Watt: %.2f", watt);
  Brain.Screen.printAt(x, y + 100,false, "Gearing: %s",
                       (gs==ratio6_1?"6:1":(gs==ratio18_1?"18:1":"36:1")));
  Brain.Screen.printAt(x, y + 120,false, "Reversed: %s", rev ? "true" : "false");

  // Toggle Reverse
  drawPillToggle_(kW - 228, headerH_ + 10, 110, 30, rev, "Reverse");

  // Reassign Port
  Brain.Screen.setFillColor(vex::color(200,230,255));
  Brain.Screen.setPenColor(black);
  Brain.Screen.drawRectangle(kW - 108, headerH_ + 10, 98, 30);
  Brain.Screen.printAt(kW - 98, headerH_ + 30, false, "%s", "Reassign");

  if (hotswap_is_active()) {
    Brain.Screen.setFillColor(vex::color(140, 210, 160));
    Brain.Screen.drawRectangle(kW - 180, headerH_ + 48, 160, 22);
    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(kW - 172, headerH_ + 64, false, "%s", "Runtime Remap ACTIVE");
  }
}

void BrainNav::drawPortPicker() {
  Brain.Screen.setFillColor(vex::color(245,245,245));
  Brain.Screen.clearScreen();

  Brain.Screen.setFillColor(vex::color(235,235,235));
  Brain.Screen.setPenColor(black);
  Brain.Screen.drawRectangle(0, 0, kW, headerH_);
  Brain.Screen.printAt(12, 20, false, "%s", "Reassign Port");

  Brain.Screen.setFillColor(vex::color(210,210,210));
  Brain.Screen.drawRectangle(10, 4, 80, headerH_ - 8);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(20, 20, false, "%s", "< Back");

  int gx = margin_;
  int gy = headerH_ + margin_;
  int gw = kW - 2 * margin_;
  int gh = kH - gy - margin_;
  const int pCols = 7, pRows = 3;
  int cellW = (gw - (pCols - 1) * gutter_) / pCols;
  int cellH = (gh - (pRows - 1) * gutter_) / pRows;

  for (int r = 0, port = 1; r < pRows; ++r) {
    for (int c = 0; c < pCols && port <= 21; ++c, ++port) {
      int x = gx + c * (cellW + gutter_);
      int y = gy + r * (cellH + gutter_);

      bool free = isPortFree_(port);
      Brain.Screen.setFillColor(free ? vex::color(120, 210, 140) : vex::color(250, 160, 180));
      Brain.Screen.setPenColor(black);
      Brain.Screen.drawRectangle(x, y, cellW, cellH);
      Brain.Screen.printAt(x + 10, y + 24, false, "P%d", port);

      const char* alias = aliasForPortRT_(port);
      if (alias) Brain.Screen.printAt(x + 10, y + 44, false, "%s", alias);
      else Brain.Screen.printAt(x + 10, y + 44, false, "%s", free ? "FREE" : "OCCUPIED");
    }
  }

  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(16, kH - 8, false,
                       "%s", "Tap a GREEN port to move this motor. Confirm required.");
}

void BrainNav::drawConfirmOverlay_(const char* title, const char* body) {
  Brain.Screen.setFillColor(vex::color(30,30,30));
  Brain.Screen.setPenColor(vex::color(30,30,30));
  Brain.Screen.drawRectangle(0,0,kW,kH);

  int cw=300,ch=140, cx=kW/2-cw/2, cy=kH/2-ch/2;
  Brain.Screen.setFillColor(vex::color(255,255,255));
  Brain.Screen.setPenColor(black);
  Brain.Screen.drawRectangle(cx,cy,cw,ch);
  Brain.Screen.printAt(cx+14, cy+24, false, "%s", title?title:"Confirm");
  Brain.Screen.printAt(cx+14, cy+56, false, "%s", body?body:"Are you sure?");

  Brain.Screen.setFillColor(vex::color(230,230,230));
  Brain.Screen.drawRectangle(kW/2 - 100, kH/2 + 24, 84, 28);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(kW/2 - 94, kH/2 + 44, false, "%s", "No");

  Brain.Screen.setFillColor(vex::color(200,230,255));
  Brain.Screen.drawRectangle(kW/2 + 16, kH/2 + 24, 84, 28);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(kW/2 + 24, kH/2 + 44, false, "%s", "Yes");
}

// ------------------------------------------------------------
//  Main UI loop
// ------------------------------------------------------------
void BrainNav::open() {
  screen_ = Screen::Home;
  selected_ = -1;
  pendingAction_ = ConfirmKind::None;
  pendingInt_ = -1;

  while (true) {
    // ----- Home -----
    if (screen_ == Screen::Home) {
      drawHome();
      while (screen_ == Screen::Home) {
        if (tappedRect_(10, 10, 80, 24)) {   // < Back
          waitForRelease_();
          Brain.Screen.clearScreen();
          return;
        }
        int bw = 160, bh = 48;
        int bx = kW/2 - bw/2;
        int by = kH/2 - bh/2;
        if (tappedRect_(bx, by, bw, bh)) {
          waitForRelease_();
          screen_ = Screen::Motors;
          break;
        }
        task::sleep(10);
      }
    }

    // ----- Motors -----
    if (screen_ == Screen::Motors) {
      uint32_t lastScan = Brain.Timer.system();
      while (screen_ == Screen::Motors) {
        uint32_t now = Brain.Timer.system();
        if (rescan_ && (now - lastScan > 1200)) {
          rescan_();
          lastScan = now;
        }
        drawMotors();

        if (tappedRect_(10, 4, 80, headerH_ - 8)) { // Back
          waitForRelease_();
          screen_ = Screen::Home;
          break;
        }

        if (tappedRect_(kW - 120, 4, 110, headerH_ - 8)) {
          waitForRelease_();
          pendingAction_ = ConfirmKind::ResetAll;
          screen_ = Screen::Confirm;
          continue;
        }

        if (Brain.Screen.pressing()) {
          int idx = indexAt_(Brain.Screen.xPosition(), Brain.Screen.yPosition());
          if (idx >= 0) {
            selected_ = idx;
            waitForRelease_();
            screen_ = Screen::MotorDetail;
            continue;
          }
        }
        task::sleep(120);
      }
    }

    // ----- Motor Detail -----
    if (screen_ == Screen::MotorDetail) {
      uint32_t lastScan = Brain.Timer.system();
      while (screen_ == Screen::MotorDetail) {
        uint32_t now = Brain.Timer.system();
        if (rescan_ && (now - lastScan > 800)) {
          rescan_();
          lastScan = now;
        }
        drawMotorDetail();

        if (tappedRect_(10, 4, 80, headerH_ - 8)) { // Back
          waitForRelease_();
          screen_ = Screen::Motors;
          break;
        }

        // Toggle reverse
        if (tappedRect_(kW - 228, headerH_ + 10, 110, 30)) {
          waitForRelease_();
          pendingAction_ = ConfirmKind::ToggleReverse;
          pendingInt_ = -1;
          screen_ = Screen::Confirm;
          continue;
        }

        // Reassign Port
        if (tappedRect_(kW - 108, headerH_ + 10, 98, 30)) {
          waitForRelease_();
          screen_ = Screen::PortPicker;
          break;
        }

        task::sleep(80);
      }
    }

    // ----- Port Picker -----
    if (screen_ == Screen::PortPicker) {
      drawPortPicker();

      if (tappedRect_(10, 4, 80, headerH_ - 8)) { // Back to detail
        waitForRelease_();
        screen_ = Screen::MotorDetail;
        continue;
      }

      if (Brain.Screen.pressing() && selected_ >= 0) {
        int px = Brain.Screen.xPosition();
        int py = Brain.Screen.yPosition();

        int gx = margin_, gy = headerH_ + margin_;
        int gw = kW - 2 * margin_;
        int gh = kH - gy - margin_;
        const int pCols = 7, pRows = 3;
        int cellW = (gw - (pCols - 1) * gutter_) / pCols;
        int cellH = (gh - (pRows - 1) * gutter_) / pRows;

        for (int r = 0, port = 1; r < pRows; ++r) {
          for (int c = 0; c < pCols && port <= 21; ++c, ++port) {
            int x = gx + c * (cellW + gutter_);
            int y = gy + r * (cellH + gutter_);
            if (px >= x && px <= x + cellW && py >= y && py <= y + cellH) {
              if (isPortFree_(port)) {
                pendingAction_ = ConfirmKind::Reassign;
                pendingInt_ = port;
                waitForRelease_();
                screen_ = Screen::Confirm;
              } else {
                waitForRelease_();
              }
              r = pRows; break;
            }
          }
        }
      }
      task::sleep(60);
    }

    // ----- Confirm -----
    if (screen_ == Screen::Confirm) {
      const char* t = "";
      const char* b = "";
      if (pendingAction_ == ConfirmKind::ToggleReverse) {
        t = "Confirm Reverse Toggle";
        b = "Toggle motor reverse?";
      } else if (pendingAction_ == ConfirmKind::Reassign) {
        t = "Confirm Reassign";
        b = "Move motor to selected port?";
      } else if (pendingAction_ == ConfirmKind::ResetAll) {
        t = "Confirm Reset All";
        b = "Reset all motors to seeded config?";
      }
      drawConfirmOverlay_(t, b);

      // NO
      if (tappedRect_(kW/2 - 100, kH/2 + 24, 84, 28)) {
        waitForRelease_();
        if (pendingAction_ == ConfirmKind::Reassign) screen_ = Screen::PortPicker;
        else if (selected_ >= 0) screen_ = Screen::MotorDetail;
        else screen_ = Screen::Motors;
        pendingAction_ = ConfirmKind::None;
        pendingInt_ = -1;
      }

      // YES
      if (tappedRect_(kW/2 + 16, kH/2 + 24, 84, 28)) {
        waitForRelease_();
        if (pendingAction_ == ConfirmKind::ToggleReverse && selected_ >= 0) {
          auto& mi = motors_[selected_];
          if (mi.alias && *mi.alias) {
            bool cur=false;
            if (hotswap_get_reverse_by_alias(mi.alias, cur)) {
              hotswap_set_reverse_by_alias(mi.alias, !cur);
            }
          }
          if (rescan_) rescan_();
          screen_ = Screen::MotorDetail;

        } else if (pendingAction_ == ConfirmKind::Reassign &&
                   selected_ >= 0 && pendingInt_ >= 1) {
          auto& mi = motors_[selected_];
          bool ok = false;
          if (mi.alias && *mi.alias) {
            ok = hotswap_reassign_by_alias(mi.alias, pendingInt_);
          } else if (mi.m) {
            delete mi.m;
            mi.m = new vex::motor(pendingInt_);
            ok = mi.m->installed();
          }
          if (ok && rescan_) rescan_();
          screen_ = Screen::MotorDetail;

        } else if (pendingAction_ == ConfirmKind::ResetAll) {
          reset_all_to_seed_();
          if (rescan_) rescan_();
          screen_ = Screen::Motors;
        }
        pendingAction_ = ConfirmKind::None;
        pendingInt_ = -1;
      }
      task::sleep(30);
    }
  }
}
