#pragma once
#include "vex.h"
#include <cmath>

namespace sdrive {

class SDrive {
public:
  SDrive(vex::motor_group& left,
         vex::motor_group& right,
         vex::inertial* imu,
         double wheelDiamIn,
         double gearRatio,
         double trackWidthIn);

  // Configure motion limits (linear)
  void setLimits(double maxV_in_s = 36.0,
                 double maxA_in_s2 = 72.0,
                 double maxJ_in_s3 = 600.0);

  // Heading PD gains
  void setTurnGains(double kp = 0.07, double kd = 0.0009); // Kp used to be 0.02 but was undershooting just a liiitle bit

  // S-curve smoothing distance (inches)
  void setSmoothing(double smoothInches = 8.0);

  // Basic moves
  void s_drive(double inches);
  void s_drive(double inches, double finalHeadingDeg);

  // Drive along an arc to a field point (x,y) using shortest heading
  void s_drive_shortest(double targetX, double targetY);

private:
  vex::motor_group& left_;
  vex::motor_group& right_;
  vex::inertial* imu_;

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

  void runProfile_(double targetInches, bool useHeading, double finalHeadingDeg);
  void commandVel_(double v_in_s, double omega_deg_s);

  static double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
  }
};

// ------------------ Global access functions ------------------
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
void s_drive_shortest(double x, double y);

} // namespace sdrive
