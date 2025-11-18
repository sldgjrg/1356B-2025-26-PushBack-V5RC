#include "Hurr-Template/s-drive.h"
#include "JAR-Template/drive.h"   // for chassis.get_X_position(), etc.
#include "vex.h"

using namespace vex;

namespace sdrive {

static SDrive* G = nullptr;

static double wrap(double deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

static constexpr double PI          = 3.14159265358979323846;
// Distance calibration: internal target = commanded * DIST_SCALE.
static constexpr double DIST_SCALE  = 0.95;

// ---------------- Constructor ----------------
SDrive::SDrive(motor_group& left,
               motor_group& right,
               inertial* imu,
               double wheelDiamIn,
               double gearRatio,
               double trackWidthIn)
  : left_(left),
    right_(right),
    imu_(imu),
    wheelDiamIn_(wheelDiamIn),
    gearRatio_(gearRatio),
    trackWidthIn_(trackWidthIn),
    maxV_(36.0),
    maxA_(72.0),
    maxJ_(600.0),
    smoothK_(8.0),
    // heading correction while moving
    kTurnP_(0.06),
    kTurnD_(0.0) {}

// ---------------- Settings ----------------
void SDrive::setLimits(double v, double a, double j) {
  maxV_ = v;
  maxA_ = a;
  maxJ_ = j;
}

void SDrive::setTurnGains(double kp, double kd) {
  kTurnP_ = kp;
  kTurnD_ = kd;
}

void SDrive::setSmoothing(double s) {
  smoothK_ = s;
}

// ---------------- Helpers ----------------
void SDrive::resetEncoders_() {
  left_.resetPosition();
  right_.resetPosition();
}

double SDrive::wheelRevToInches_(double rev) const {
  return rev * (PI * wheelDiamIn_) * gearRatio_;
}

double SDrive::avgDistanceIn_() const {
  double L = left_.position(rotationUnits::rev);
  double R = right_.position(rotationUnits::rev);
  return wheelRevToInches_((L + R) * 0.5);
}

// Use continuous rotation
double SDrive::headingDeg_() const {
  return imu_ ? imu_->rotation() : 0.0;
}

// Convert linear + angular → wheel speeds
void SDrive::commandVel_(double v_in_s, double omega_deg_s) {
  double omega = omega_deg_s * PI / 180.0;
  double vDiff = -omega * (trackWidthIn_ / 2.0);

  double vL = v_in_s - vDiff;
  double vR = v_in_s + vDiff;

  auto toRPM = [&](double v) {
    return (v / (PI * wheelDiamIn_)) * 60.0 / gearRatio_;
  };

  left_.spin(fwd, toRPM(vL), velocityUnits::rpm);
  right_.spin(fwd, toRPM(vR), velocityUnits::rpm);
}

// ---------------- Motion Profile (geometric arc + scaled angle & distance) ----------------
void SDrive::runProfile_(double targetInches,
                         bool useHeading,
                         double finalHeadingDeg,
                         bool stopAtEnd) {

  resetEncoders_();

  // Distance scaling: internally aim a bit different so field distance matches better.
  double rawD = fabs(targetInches);
  double D    = rawD * DIST_SCALE;
  double sign = (targetInches >= 0.0) ? 1.0 : -1.0;

  if (D < 0.05) {
    if (stopAtEnd) {
      left_.stop(brakeType::hold);
      right_.stop(brakeType::hold);
    }
    return;
  }

  const double dt          = 0.01;
  const int    dt_ms       = 10;

  const double distTol     = 0.25;  // inches
  const double headingDead = 0.5;   // deg: ignore tiny error near the end

  // Heading geometry
  double h0       = headingDeg_();
  double rawDelta = 0.0;  // real target change
  double arcDelta = 0.0;  // exaggerated change for arc shape
  double hGoal    = h0;   // actual angle we want to end near (for shaping only now)

  if (useHeading && imu_) {
    rawDelta = wrap(finalHeadingDeg - h0);
    const double ARC_SCALE = 1.3; // scalar to fight angular undershoot / start arc earlier
    arcDelta = rawDelta * ARC_SCALE;
    hGoal    = h0 + rawDelta;
  }

  // Geometric curvature: Δθ_arc / D (radians per inch)
  double curvature = 0.0;
  if (useHeading && fabs(arcDelta) > 0.5 && D > 1.0) {
    double deltaH_rad = arcDelta * PI / 180.0;
    curvature = deltaH_rad / D;    // rad / inch
  } else {
    curvature = 0.0; // effectively straight or tiny heading change
  }

  double v_cmd   = 0.0;
  double a_cmd   = 0.0;
  double prevErr = 0.0;

  while (true) {

    double x   = fabs(avgDistanceIn_());
    double rem = D - x;
    if (rem < 0) rem = 0;

    // 👉 EXIT CONDITION: only distance. Once we're there, we are DONE.
    if (rem <= distTol)
      break;

    double hMeas = headingDeg_();

    double p = (D > 0.01) ? clamp(x / D, 0.0, 1.0) : 1.0;

    // Smooth S-curve speed
    double v_target = maxV_ * tanh(rem / smoothK_);
    v_target *= sign;

    // Limit acceleration/jerk
    double desired_a = (v_target - v_cmd) / dt;
    double da = clamp(desired_a - a_cmd, -maxJ_ * dt, maxJ_ * dt);

    a_cmd += da;
    a_cmd = clamp(a_cmd, -maxA_, maxA_);
    v_cmd += a_cmd * dt;

    double omega_cmd = 0.0;

    if (useHeading && imu_) {
      // Geometric open-loop arc using arcDelta
      double omega_ff_rad = v_cmd * curvature;               // rad/s
      double omega_ff_deg = omega_ff_rad * 180.0 / PI;       // deg/s

      // Expected heading along the ARC (scaled)
      double expectedH = h0 + ( (D > 0.01) ? (x / D) * arcDelta : arcDelta );
      double err       = wrap(expectedH - hMeas);
      double deriv     = (err - prevErr) / dt;
      prevErr          = err;

      // Deadband so we don't micro-correct near the end
      if (fabs(err) < headingDead && p > 0.8) {
        err   = 0.0;
        deriv = 0.0;
      }

      double omega_fb  = kTurnP_ * err + kTurnD_ * deriv;

      omega_cmd = omega_ff_deg + omega_fb;

      // Clamp to reasonable turn speed
      const double maxTurn = 230.0;
      if (omega_cmd >  maxTurn) omega_cmd =  maxTurn;
      if (omega_cmd < -maxTurn) omega_cmd = -maxTurn;
    }

    commandVel_(v_cmd, omega_cmd);
    wait(dt_ms, msec);
  }

  // We decided we are DONE; just stop and move on
  if (stopAtEnd) {
    left_.stop(brakeType::hold);
    right_.stop(brakeType::hold);
  } else {
    left_.stop(brakeType::coast);
    right_.stop(brakeType::coast);
  }
}

// ---------------- Public Single-Move API ----------------
void SDrive::s_drive(double inches) {
  runProfile_(inches, false, 0.0, true);
}

void SDrive::s_drive(double inches, double finalHeadingDeg) {
  runProfile_(inches, true, finalHeadingDeg, true);
}

// ---------------- Chained S-Drive (smooth succession) ----------------
void SDrive::s_drive_chain(std::initializer_list<Command> cmds) {

  if (cmds.size() == 0) return;

  size_t idx = 0;
  size_t n   = cmds.size();

  for (const auto& cmd : cmds) {
    bool last = (idx == n - 1);

    runProfile_(cmd.inches,
                cmd.useHeading,
                cmd.headingDeg,
                last);          // only last step does a firm hold

    idx++;
  }
}

// ---------------- Drive to X,Y using shortest heading ----------------
void SDrive::s_drive_shortest(double targetX, double targetY) {

  double curX = chassis.get_X_position();
  double curY = chassis.get_Y_position();

  double dx = targetX - curX;
  double dy = targetY - curY;

  double distance = sqrt(dx*dx + dy*dy);
  double heading  = wrap(atan2(dy, dx) * 180.0 / PI);

  runProfile_(distance, true, heading, true);
}

// ---------------- Global wrappers ----------------
void init(motor_group& left,
          motor_group& right,
          inertial* imu,
          double wheelDiamIn,
          double gearRatio,
          double trackWidthIn) {
  static SDrive inst(left, right, imu, wheelDiamIn, gearRatio, trackWidthIn);
  G = &inst;
}

void set_limits(double v, double a, double j) {
  if (G) G->setLimits(v,a,j);
}

void set_turn_gains(double kp, double kd) {
  if (G) G->setTurnGains(kp,kd);
}

void set_smoothing(double s) {
  if (G) G->setSmoothing(s);
}

void s_drive(double x) {
  if (G) G->s_drive(x);
}

void s_drive(double x, double deg) {
  if (G) G->s_drive(x, deg);
}

void s_drive_chain(std::initializer_list<Command> cmds) {
  if (G) G->s_drive_chain(cmds);
}

void s_drive_shortest(double x, double y) {
  if (G) G->s_drive_shortest(x, y);
}

} // namespace sdrive
