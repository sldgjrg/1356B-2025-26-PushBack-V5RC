#pragma once
#include "vex.h"

using namespace vex;

/* ============================================================
   FULL MOTOR MANAGER RUNTIME + BRAIN UI
   All declarations for runtime tables + BrainNav class
   ============================================================ */

// ---------- RUNTIME TABLES (motors, alias, config) ----------

// Runtime record for 1 alias-bound motor
struct MotorRuntime {
    gearSetting gs;
    bool reversed;
    int port;               // 1..21
};

// Binding alias → vex::motor*
struct Binding {
    const char* name;       // alias name (string literal)
    vex::motor* ref;        // pointer to the actual motor instance
};

// Global runtime + binding tables
extern Binding      gBindings[32];
extern int          gBindingCount;
extern MotorRuntime gRuntime[32];
extern int          gRuntimeCount;

// ---------- Runtime manipulation helpers ----------
bool hotswap_reassign_by_alias(const char* aliasName, int newPort);
bool hotswap_set_reverse_by_alias(const char* aliasName, bool newVal);
bool hotswap_get_reverse_by_alias(const char* aliasName, bool& outVal);
int  hotswap_get_port_by_alias(const char* aliasName);
bool hotswap_get_gearing_by_alias(const char* aliasName, gearSetting& outGs);
bool hotswap_is_active();

// System setup
void buildBindings();              // alias→motor* mapping
void seedRuntimeFromConfig();      // read motor-config initial state
void discoverMotors();             // scan physical installed motors

// Reset runtime table back to seed
void reset_all_to_seed_();

// ---------- UI STRUCT ----------
struct MotorInfo {
    vex::motor* m;      // pointer to motor (for temperature, installed state)
    const char* name;   // visual name (“LeftFront”)
    const char* alias;  // alias match (same as name for extern motors)
};

// ============================================================
//                         BRAIN NAV UI
// ============================================================
class BrainNav {
public:
    struct TempThresholds {
        double yellow;
        double red;
        constexpr TempThresholds(double y=45.0, double r=55.0)
            : yellow(y), red(r) {}
    };

    BrainNav(MotorInfo* motors,
             int*       countRef,
             TempThresholds temp = TempThresholds{},
             void (*rescanCb)() = nullptr);

    void open();    // open UI (blocking)

private:
    enum class Screen { Home, Motors, MotorDetail, PortPicker, Confirm };
    enum class ConfirmKind { None, ToggleReverse, Reassign, ResetAll };

    // ----- state -----
    Screen screen_ = Screen::Home;
    ConfirmKind pendingAction_ = ConfirmKind::None;
    int pendingInt_ = -1;
    int selected_ = -1;

    MotorInfo* motors_;
    int* countRef_;
    TempThresholds tt_;
    void (*rescan_)() = nullptr;

    // layout
    int cols_ = 4;
    int margin_ = 6;
    int gutter_ = 6;
    int headerH_ = 28;

    // ---------- internal methods ----------
    // drawing
    void drawHome();
    void drawMotors();
    void drawMotorDetail();
    void drawPortPicker();
    void drawConfirmOverlay_(const char* title, const char* body);

    // touch
    bool tappedRect_(int x, int y, int w, int h);
    bool waitForRelease_();
    int  indexAt_(int px, int py) const;

    // utils
    vex::color tempToColor_(double c) const;
    vex::color darken_(const vex::color& c, double k) const;
    vex::color penForTemp_(double c) const;
    const char* abbrev_(const char* name, char* buf, int bufsz) const;

    // grid geometry
    void gridGeom_(int& gx, int& gy, int& gw, int& gh, int& cellW, int& cellH, int& rows) const;

    // reverse/port tools
    bool isPortFree_(int port) const;
    const char* aliasForPortRT_(int port) const;

    // widgets
    void drawPillToggle_(int x, int y, int w, int h, bool on, const char* label);
};
