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
static constexpr double DEADZONE = 0.35;
static constexpr double T_MIN = 0.15;

// ----------------------------------------------------------
SDrive::SDrive(motor_group& left,
               motor_group& right,
               inertial* imu,
               double wheelDiamIn,
               double gearRatio,
               double trackWidthIn)
  : left_(left), right_(right), imu_(imu),
    wheelDiamIn_(wheelDiamIn),
    gearRatio_(gearRatio),
    trackWidthIn_(trackWidthIn),
    maxV_(140),            // insane speed
    maxA_(600),            // insane accel
    maxJ_(6000),           // unused but OK
    smoothK_(8),
    kTurnP_(0.06),
    kTurnD_(0.0)
{}

// ----------------------------------------------------------
void SDrive::setLimits(double v,double a,double j){
  maxV_ = v;
  maxA_ = a;
  maxJ_ = j;
}

void SDrive::setTurnGains(double kp,double kd){
  kTurnP_ = kp;
  kTurnD_ = kd;
}

void SDrive::setSmoothing(double s){
  smoothK_ = s;
}

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

  auto toRPM=[&](double v){
    return (v/(PI*wheelDiamIn_))*60.0/gearRatio_;
  };

  left_.spin(fwd,toRPM(vL),velocityUnits::rpm);
  right_.spin(fwd,toRPM(vR),velocityUnits::rpm);
}

// ----------------------------------------------------------
// Hybrid Racing Trajectory
// ----------------------------------------------------------
void SDrive::runProfile_(double targetInches,
                         bool useHeading,
                         double finalHeading,
                         bool stopAtEnd)
{
  resetEncoders_();

  double rawD = fabs(targetInches);
  double D    = rawD * DIST_SCALE;
  double sign = (targetInches >= 0 ? 1 : -1);

  if (D < 0.05){
    commandVel_(0,0);
    return;
  }

  const double dt = 0.01;
  const int dtms  = 10;

  // ---------------- ARC TURN CALC ----------------
  double h0 = headingDeg_();
  double arcDelta = 0;

  if (useHeading && imu_) {
    arcDelta = wrap(finalHeading - h0) * 1.3;
  }

  double curvature = 0;
  if (fabs(arcDelta) > 0.5 && D > 1.0)
    curvature = (arcDelta * PI/180.0) / D;

  // ---------------- TIMING ----------------
  // aggressive timing: fast accel, fast cruise
  double T_accel = maxV_ / maxA_;     // time to reach maxV
  double T_total = std::max(T_accel * 2.0, T_MIN);

  double t = 0;
  double prevErr = 0;

  // ----------------------------------------------------------
  // MAIN LOOP
  // ----------------------------------------------------------
  while(true){

    double x = fabs(avgDistanceIn_());
    double rem = D - x;

    if (rem <= DEADZONE || t >= T_total)
      break;

    double v_accel = maxV_ * (1.0 - exp(-6.0 * (t / T_accel)));
    if (v_accel > maxV_) v_accel = maxV_;

    double t_dec = (T_total - t) / T_total;
    double v_slow = maxV_ / (1.0 + exp(12.0 * (t_dec - 0.5)));

    double v_mag = std::min(v_accel, v_slow);

    if (v_mag > maxV_) v_mag = maxV_;

    double v_cmd = sign * v_mag;

    double omega_cmd = 0;

    if (useHeading && imu_){
      double p = 1.0 - (rem / D);
      double expectedH = h0 + arcDelta * p;

      double err = wrap(expectedH - headingDeg_());
      double deriv=(err - prevErr)/dt;
      prevErr=err;

      if (fabs(v_cmd) < 8 || rem < 3){
        err = deriv = 0;
      }

      double omega_ff = v_cmd * curvature * 180.0/PI;
      double omega_fb = kTurnP_*err + kTurnD_*deriv;

      omega_cmd = clamp(omega_ff + omega_fb, -240, 240);
    }

    commandVel_(v_cmd, omega_cmd);

    wait(dtms, msec);
    t += dt;
  }

  // ---------------- SMOOTH BRAKE ----------------
  commandVel_(0,0);
  left_.stop(brakeType::coast);
  right_.stop(brakeType::coast);

  wait(120, msec);

  if(stopAtEnd){
    left_.stop(brakeType::hold);
    right_.stop(brakeType::hold);
  }
}

// ----------------------------------------------------------
void SDrive::s_drive(double inches){
  runProfile_(inches,false,0,true);
}
void SDrive::s_drive(double inches,double heading){
  runProfile_(inches,true,heading,true);
}

void SDrive::s_drive_chain(std::initializer_list<Command> cmds){
  size_t idx=0,n=cmds.size();
  for(auto &c:cmds){
    runProfile_(c.inches, c.useHeading, c.headingDeg, idx==n-1);
    idx++;
  }
}

void SDrive::s_drive_shortest(double tx,double ty){
  double cx = chassis.get_X_position();
  double cy = chassis.get_Y_position();

  double dx = tx - cx;
  double dy = ty - cy;

  double d = std::sqrt(dx*dx + dy*dy);
  double h = wrap(std::atan2(dy,dx)*180.0/PI);

  runProfile_(d,true,h,true);
}

// ----------------------------------------------------------
void init(motor_group& left,motor_group& right,inertial* imu,
          double wd,double gr,double tw){
  static SDrive inst(left,right,imu,wd,gr,tw);
  G=&inst;
}

void set_limits(double v,double a,double j){ if(G)G->setLimits(v,a,j); }
void set_turn_gains(double kp,double kd){ if(G)G->setTurnGains(kp,kd); }
void set_smoothing(double s){ if(G)G->setSmoothing(s); }
void s_drive(double i){ if(G)G->s_drive(i); }
void s_drive(double i,double h){ if(G)G->s_drive(i,h); }
void s_drive_chain(std::initializer_list<Command> c){ if(G)G->s_drive_chain(c); }
void s_drive_shortest(double x,double y){ if(G)G->s_drive_shortest(x,y); }

} // namespace sdrive
