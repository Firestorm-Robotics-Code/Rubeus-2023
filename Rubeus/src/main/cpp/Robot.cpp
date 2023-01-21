/* By Tyler Clarke
  This is an experiment with c++20 features and new paradigms to make FRC robot code cleaner.
  The idea: structuring an FRC robot like a real C++ program instead of like Java gone even wronger. Craszy.
*/
#include <FRL/bases/AwesomeRobotBase.hpp>
#include <FRL/motor/SparkMotor.hpp>
#include <FRL/swerve/SwerveModule.hpp>
#include <constants.h>
#include <frc/XboxController.h>
#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define PI              3.141592
#define XBOX_DEADBAND   0.05
#define XBOX_SPEEDLIMIT 0.2


SwerveModule frontLeftSwerve (
  new SparkMotor(FRONT_LEFT_SPEED),
  new SparkMotor(FRONT_LEFT_DIREC), 
  FRONT_LEFT_CANCODER,
  1, 
  2048 + FRONT_LEFT_OFFSET
);

SwerveModule frontRightSwerve (
  new SparkMotor(FRONT_RIGHT_SPEED),
  new SparkMotor(FRONT_RIGHT_DIREC),
  FRONT_RIGHT_CANCODER,
  2, 
  2048 + FRONT_RIGHT_OFFSET
);

SwerveModule mainSwerve (
  new SparkMotor(BACK_LEFT_SPEED),
  new SparkMotor(BACK_LEFT_DIREC),
  BACK_LEFT_CANCODER,
  4, 
  BACK_LEFT_OFFSET
);

SwerveModule backRightSwerve (
  new SparkMotor(BACK_RIGHT_SPEED),
  new SparkMotor(BACK_RIGHT_DIREC),
  BACK_RIGHT_CANCODER,
  3, 
  BACK_RIGHT_OFFSET
);


frc::XboxController xbox {4};
AHRS navx {frc::SPI::Port::kMXP}; // Well, obviously, the navx
double navxOffset = 0;

long navxHeadingToEncoderTicks(){
  return (navx.GetFusedHeading() - navxOffset) * 4096/360;
}

void zeroNavx(){
  navxOffset = navx.GetFusedHeading();
}


class TeleopMode : public RobotMode {
public:
  void Start(){
    zeroNavx();
  }

  void Synchronous(){
    long direction = navxHeadingToEncoderTicks();
    float dx = xbox.GetLeftX();
    float dy = xbox.GetLeftY();
    int xboxPOV = xbox.GetPOV();
    float hypotenuse = sqrt(dx * dx + dy * dy);
    hypotenuse *= XBOX_SPEEDLIMIT;
    double joystickDir = atan(dy/dx);
    if (dx < 0){
      joystickDir += PI;
    }
    joystickDir = PI - joystickDir; // Flip it? I think?
    direction += joystickDir * 4096/(2 * PI);
    if ((dx * dx + dy * dy) > XBOX_DEADBAND * XBOX_DEADBAND){
      mainSwerve.SetDirection(smartLoop(direction)); // 0 the entire drive
      mainSwerve.MovePercent(hypotenuse);
    }
    else if (xboxPOV != 0) {
      mainSwerve.Orient(XboxPOV);
    }
    else{
      mainSwerve.SetDirection(0);
      mainSwerve.MovePercent(0);
    }
    mainSwerve.ApplySpeed();
  }
};


class AutonomousMode : public RobotMode {

};


class TestMode : public RobotMode {

};


class DisabledMode : public RobotMode {

};


#ifndef RUNNING_FRC_TESTS // I'm afraid to remove this.
int main() {
  mainSwerve.Link(&backRightSwerve); // Weird, right? This can in fact be used here.
  backRightSwerve.Link(&frontRightSwerve);
  frontRightSwerve.Link(&frontLeftSwerve);
  // As it turns out, int main actually still exists and even works here in FRC. I'm tempted to boil it down further and get rid of that stupid StartRobot function (replace it with something custom inside AwesomeRobot).
  return frc::StartRobot<AwesomeRobot<TeleopMode, AutonomousMode, TestMode, DisabledMode>>(); // Look, the standard library does these nested templates more than I do.
}
#endif
