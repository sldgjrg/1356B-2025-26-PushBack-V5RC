#include "Hurr-Template/s-drive.h"
#include "JAR-Template/drive.h"   // for chassis.get_X_position(), etc.
#include "vex.h"

using namespace vex;

namespace sdrive {

static SDrive* G = nullptr;

// ---------- Helper: wrap to [-180,180] ----------
static double wrap(double deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

// ---------------- Constructor -------------------
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
    kTurnP_(0.02),
    kTurnD_(0.0009) {}

// ------------------- Settings functions --------------------

void SDrive::setLimits(double maxV_in_s,
                       double maxA_in_s2,
                       double maxJ_in_s3) {
  maxV_ = maxV_in_s;
  maxA_ = maxA_in_s2;
  maxJ_ = maxJ_in_s3;
}

void SDrive::setTurnGains(double kp, double kd) {
  kTurnP_ = kp;
  kTurnD_ = kd;
}

void SDrive::setSmoothing(double smoothInches) {
  smoothK_ = smoothInches;
}

// ------------------ Small helpers ---------------------
void SDrive::resetEncoders_() {
  left_.resetPosition();
  right_.resetPosition();
}

double SDrive::wheelRevToInches_(double rev) const {
  return rev * (M_PI * wheelDiamIn_) * gearRatio_;
}

double SDrive::avgDistanceIn_() const {
  double L = left_.position(rotationUnits::rev);
  double R = right_.position(rotationUnits::rev);
  return wheelRevToInches_((L + R) * 0.5);
}

double SDrive::headingDeg_() const {
  // IMU heading is 0..360, CCW+. We let wrap() handle normalization.
  return imu_ ? imu_->heading() : 0.0;
}

// ---------------- S-curve motion profile ----------------
void SDrive::runProfile_(double targetInches, bool useHeading, double finalHeadingDeg) {

  resetEncoders_();

  double D = fabs(targetInches);
  double sign = (targetInches >= 0) ? 1.0 : -1.0;

  if (D < 0.1) return;

  // Starting and final angle
  double h0 = wrap(headingDeg_());
  double hFinal = h0;

  if (useHeading && imu_) {
    double goal = wrap(finalHeadingDeg);
    // shortest-path difference
    double diff = wrap(goal - h0);
    hFinal = h0 + diff;
  }

  const int dt_ms = 10;
  const double dt = 0.01;

  double v_cmd = 0, a_cmd = 0;
  double prevErr = 0;

  while (true) {

    double x = fabs(avgDistanceIn_());
    double rem = D - x;

    if (rem <= 0.05)
      break;

    double p = clamp(x / D, 0.0, 1.0);

    // Smooth S-curve speed (tanh)
    double v_target = maxV_ * tanh(rem / smoothK_);
    v_target *= sign;

    double desired_a = (v_target - v_cmd) / dt;
    double da = clamp(desired_a - a_cmd, -maxJ_ * dt, maxJ_ * dt);

    a_cmd += da;
    a_cmd = clamp(a_cmd, -maxA_, maxA_);
    v_cmd += a_cmd * dt;

    // Heading control
    double omega_cmd = 0;

    if (useHeading && imu_) {

      double S  = 3*p*p - 2*p*p*p;
      double dS = 6*p - 6*p*p;

      double hRef = h0 + (hFinal - h0) * S;
      double hMeas = wrap(headingDeg_());
      double err = wrap(hRef - hMeas);

      double dpdt = (D > 0.01) ? fabs(v_cmd) / D : 0.0;
      double hFF = (hFinal - h0) * dS * dpdt;

      omega_cmd = kTurnP_ * err + kTurnD_ * ((err - prevErr) / dt) + hFF;
      prevErr = err;
    }

    commandVel_(v_cmd, omega_cmd);
    wait(dt_ms, msec);
  }

  left_.stop(brakeType::hold);
  right_.stop(brakeType::hold);
}

// ---------- Convert linear + angular vel → wheel speeds ---------
void SDrive::commandVel_(double v_in_s, double omega_deg_s) {

  double omega = omega_deg_s * M_PI / 180.0;
  // NOTE: negative sign so positive omega turns correct direction
  double vDiff = -omega * (trackWidthIn_ / 2);

  double vL = v_in_s - vDiff;
  double vR = v_in_s + vDiff;

  auto toRPM = [&](double v) {
    return (v / (M_PI * wheelDiamIn_)) * 60.0 / gearRatio_;
  };

  left_.spin(fwd, toRPM(vL), velocityUnits::rpm);
  right_.spin(fwd, toRPM(vR), velocityUnits::rpm);
}

// ---------------- Public functions ---------------------

void SDrive::s_drive(double inches) {
  runProfile_(inches, false, 0);
}

void SDrive::s_drive(double inches, double finalHeadingDeg) {
  runProfile_(inches, true, finalHeadingDeg);
}

// ⭐ Drive to X,Y using shortest-turn arc
void SDrive::s_drive_shortest(double targetX, double targetY) {

  double curX = chassis.get_X_position();
  double curY = chassis.get_Y_position();

  double dx = targetX - curX;
  double dy = targetY - curY;

  double distance = sqrt(dx*dx + dy*dy);
  double heading = wrap(atan2(dy, dx) * 180.0 / M_PI);

  runProfile_(distance, true, heading);
}

// ---------------- Global wrappers ---------------------
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

void s_drive_shortest(double x, double y) {
  if (G) G->s_drive_shortest(x, y);
}

} // namespace sdrive
