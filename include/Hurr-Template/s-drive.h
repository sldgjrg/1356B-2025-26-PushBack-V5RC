#pragma once
#include "vex.h"
#include <cmath>
#include <initializer_list>

namespace sdrive {

struct Command {
  double inches;
  double headingDeg;
  bool   useHeading;

  Command(double dist)
    : inches(dist), headingDeg(0.0), useHeading(false) {}

  Command(double dist, double heading)
    : inches(dist), headingDeg(heading), useHeading(true) {}
};

class SDrive {
public:
  SDrive(vex::motor_group& left,
         vex::motor_group& right,
         vex::inertial* imu,
         double wheelDiamIn,
         double gearRatio,
         double trackWidthIn);

  void setLimits(double maxV_in_s  = 80.0,
                 double maxA_in_s2 = 250.0,
                 double maxJ_in_s3 = 3000.0);

  void setTurnGains(double kp = 0.06, double kd = 0.0);
  void setSmoothing(double smoothInches = 8.0);

  void s_drive(double inches);
  void s_drive(double inches, double finalHeadingDeg);

  void s_drive_chain(std::initializer_list<Command> cmds);
  void s_drive_shortest(double targetX, double targetY);

private:
  vex::motor_group& left_;
  vex::motor_group& right_;
  vex::inertial*    imu_;

  double wheelDiamIn_;
  double gearRatio_;
  double trackWidthIn_;

  double maxV_;
  double maxA_;
  double maxJ_;
  double smoothK_;

  double kTurnP_;
  double kTurnD_;

  void resetEncoders_();
  double wheelRevToInches_(double rev) const;
  double avgDistanceIn_() const;
  double headingDeg_() const;

  void runProfile_(double targetInches,
                   bool useHeading,
                   double finalHeadingDeg,
                   bool stopAtEnd);

  void commandVel_(double v_in_s, double omega_deg_s);

  static double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
  }
};

void init(vex::motor_group& left,
          vex::motor_group& right,
          vex::inertial* imu = nullptr,
          double wheelDiamIn = 3.25,
          double gearRatio   = 0.8,
          double trackWidthIn= 11.0);

void set_limits(double maxV_in_s, double maxA_in_s2, double maxJ_in_s3);
void set_turn_gains(double kp, double kd);
void set_smoothing(double smoothInches);

void s_drive(double inches);
void s_drive(double inches, double finalHeadingDeg);
void s_drive_chain(std::initializer_list<Command> cmds);
void s_drive_shortest(double x, double y);

} // namespace sdrive
