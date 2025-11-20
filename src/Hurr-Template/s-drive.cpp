#include "Hurr-Template/s-drive.h"
#include "JAR-Template/drive.h"
#include "vex.h"
#include <cmath>

using namespace vex;

namespace sdrive {

static SDrive* G = nullptr;

static double wrap(double deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

static constexpr double PI = 3.14159265358979323846;
static constexpr double DIST_SCALE = 0.95;
static constexpr double T_MIN      = 0.35;
static constexpr double DEADZONE   = 0.50;

// ----------------------------------------------------------
// Constructor
// ----------------------------------------------------------
SDrive::SDrive(motor_group& left,
               motor_group& right,
               inertial* imu,
               double wheelDiamIn,
               double gearRatio,
               double trackWidthIn)
  : left_(left), right_(right), imu_(imu),
    wheelDiamIn_(wheelDiamIn), gearRatio_(gearRatio), trackWidthIn_(trackWidthIn),
    maxV_(80.0), maxA_(250.0), maxJ_(3000.0),
    smoothK_(8.0),
    kTurnP_(0.06), kTurnD_(0.0)
{}

// ----------------------------------------------------------
void SDrive::setLimits(double v,double a,double j){
  maxV_ = v;
  maxA_ = a;
  maxJ_ = j;
}
void SDrive::setTurnGains(double kp,double kd){ kTurnP_ = kp; kTurnD_ = kd; }
void SDrive::setSmoothing(double s){ smoothK_ = s; }

// ----------------------------------------------------------
void SDrive::resetEncoders_(){
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

double SDrive::headingDeg_() const {
  return imu_ ? imu_->rotation() : 0.0;
}

void SDrive::commandVel_(double v_in_s,double omega_deg_s){
  double omega = omega_deg_s * PI/180.0;
  double vDiff = omega * (trackWidthIn_/2.0);

  double vL = v_in_s - vDiff;
  double vR = v_in_s + vDiff;

  auto toRPM = [&](double x){
    return (x/(PI*wheelDiamIn_))*60.0/gearRatio_;
  };

  left_.spin(fwd, toRPM(vL), velocityUnits::rpm);
  right_.spin(fwd,toRPM(vR), velocityUnits::rpm);
}

// ----------------------------------------------------------
// TANH S-CURVE + BOOSTED ROCKET TAKEOFF
// ----------------------------------------------------------
void SDrive::runProfile_(double targetInches,
                         bool useHeading,
                         double finalHeading,
                         bool stopAtEnd){

  resetEncoders_();

  double rawD = fabs(targetInches);
  double D    = rawD * DIST_SCALE;
  double sign = (targetInches >= 0) ? 1.0 : -1.0;

  if (D < 0.05){
    commandVel_(0,0);
    left_.stop(brakeType::coast);
    right_.stop(brakeType::coast);
    return;
  }

  const double dt = 0.01;
  const int dtms  = 10;

  // ------- Heading geometry --------
  double h0 = headingDeg_();
  double arcDelta = 0;

  if (useHeading && imu_) {
    arcDelta = wrap(finalHeading - h0) * 1.3;
  }

  double curvature = 0;
  if (fabs(arcDelta) > 0.5 && D > 1.0)
    curvature = (arcDelta * PI/180.0) / D;

  // ------- TANH PARAMETERS --------
  const double k = 3.0;   // main curve steepness

  auto f = [k](double u){
    double x = k*(1 - 2*u);
    return 0.5 * (1 - std::tanh(x));
  };

  auto fp = [k](double u){
    double x = k*(1 - 2*u);
    double c = std::cosh(x);
    return k / (c*c);
  };

  double f0 = f(0);
  double f1 = f(1);
  double fSpan = f1 - f0;

  // Duration based on velocity + accel limits
  double T_v = D * k / (maxV_ * fSpan);
  double T_a = std::sqrt(std::max(2.0 * D / maxA_, 0.01));
  double T   = std::max({T_v, T_a, T_MIN});

  double t = 0.0;
  double prevErr = 0.0;

  // ----------------------------------------------------------
  // MAIN LOOP
  // ----------------------------------------------------------
  while(true){

    double x   = fabs(avgDistanceIn_());
    double rem = D - x;
    if (rem < 0) rem = 0;

    if (t >= T || rem <= DEADZONE) break;

    double u = t/T;
    u = (u < 0 ? 0 : (u > 1 ? 1 : u));

    // Base tanh velocity
    double v_mag = (D/fSpan) * fp(u) / T;

    // ------------------------------------------------------
    // 🔥 ROCKET BOOST: triple acceleration during first 20%
    // ------------------------------------------------------
    if (u < 0.20) {
      double boost = (0.20 - u)/0.20;  // fades 1→0
      v_mag *= (1.0 + 2.5 * boost);    // 250% more accel at start
    }

    if (v_mag > maxV_) v_mag = maxV_;

    double v_cmd = v_mag * sign;

    // ---- Heading correction ----
    double omega_cmd = 0;
    if (useHeading && imu_) {

      double p = (D > 0.01) ? std::min(1.0, x/D) : 1.0;
      double expectedH = h0 + arcDelta * p;

      double err = wrap(expectedH - headingDeg_());
      double deriv = (err - prevErr)/dt;
      prevErr = err;

      // turn off correction at low speed
      if (fabs(v_cmd) < 8 || rem < 3) {
        err = deriv = 0;
      }

      double omega_ff = v_cmd * curvature * 180.0/PI;
      double omega_fb = kTurnP_*err + kTurnD_*deriv;

      omega_cmd = clamp(omega_ff + omega_fb, -230, 230);
    }

    commandVel_(v_cmd, omega_cmd);

    wait(dtms, msec);
    t += dt;
  }

  // ----------------------------------------------------------
  // SMOOTH END
  // ----------------------------------------------------------
  commandVel_(0,0);
  left_.stop(brakeType::coast);
  right_.stop(brakeType::coast);

  wait(200, msec);

  if (stopAtEnd){
    left_.stop(brakeType::hold);
    right_.stop(brakeType::hold);
  }
}

// ---------------------------------------------------------------------------
void SDrive::s_drive(double inches){
  runProfile_(inches, false, 0.0, true);
}
void SDrive::s_drive(double inches, double finalHeadingDeg){
  runProfile_(inches, true, finalHeadingDeg, true);
}

void SDrive::s_drive_chain(std::initializer_list<Command> cmds){
  size_t idx=0, n=cmds.size();
  for (auto& c : cmds){
    runProfile_(c.inches, c.useHeading, c.headingDeg, (idx==n-1));
    idx++;
  }
}

void SDrive::s_drive_shortest(double tx,double ty){
  double cx = chassis.get_X_position();
  double cy = chassis.get_Y_position();

  double dx = tx - cx;
  double dy = ty - cy;

  double dist = std::sqrt(dx*dx + dy*dy);
  double h = wrap(std::atan2(dy,dx)*180.0/PI);

  runProfile_(dist, true, h, true);
}

// ---------------------------------------------------------------------------
void init(vex::motor_group& left,motor_group& right,inertial* imu,
          double wd,double gr,double tw){
  static SDrive inst(left,right,imu,wd,gr,tw);
  G = &inst;
}

void set_limits(double v,double a,double j){ if(G) G->setLimits(v,a,j); }
void set_turn_gains(double kp,double kd){ if(G) G->setTurnGains(kp,kd); }
void set_smoothing(double s){ if(G) G->setSmoothing(s); }
void s_drive(double x){ if(G) G->s_drive(x); }
void s_drive(double x,double d){ if(G) G->s_drive(x,d); }
void s_drive_chain(std::initializer_list<Command> c){ if(G) G->s_drive_chain(c); }
void s_drive_shortest(double x,double y){ if(G) G->s_drive_shortest(x,y); }

} // namespace sdrive
