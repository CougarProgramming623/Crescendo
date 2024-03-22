#include "Vision.h"
#include "Robot.h"
#include "Util.h"

#include <math.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
//#include "wpi/span.h"
//#include <LimelightHelpers.h>
#include <frc/geometry/Rotation2d.h>
#include <ntcore.h>

using namespace frc;

Vision::Vision(){}
void Vision::VisionInit(){
  
}

void Vision::PrintValues() {
  double x = (double)m_AbsolutePose.X();
  double y = (double)m_AbsolutePose.Y();

  if(x != 0 && y != 0){
    DebugOutF("tx: " + std::to_string(x));
    DebugOutF("ty: " + std::to_string(y));
  }

  DebugOutF("Robot Angle: " + std::to_string(Robot::GetRobot()->GetNavX().GetAngle()));
  DebugOutF("April Tag ID: " + std::to_string(m_LimelightTable->GetNumber("tid", 0.0)));
  DebugOutF("Target Robot Angle: " + std::to_string(Rotation2d(Robot::GetRobot()->GetVision().VisionRobotYaw(m_LimelightTable->GetNumber("tid", 0.0))).Degrees().value()));
}

void Vision::setPriority(int id) {
  if(id == 3 || id == 7) {
    m_LimelightTable->GetEntry("priorityid").SetInteger(id + 1);
  }
}


void Vision::CalcPose(){
  // DebugOutF("calc Pose");
  if(m_LimelightTable->GetNumber("tv", 0.0) == 1 && m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).size() != 0){

    //print size
    //DebugOutF("size: " + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).size()));
    // DebugOutF("Tv: "   + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_TV).GetInteger(0)));


    // //print x, y, theta
    DebugOutF("\nx: " + std::to_string((int)m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).at(1)));
    DebugOutF("y: " + std::to_string((int)m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).at(0)));
    DebugOutF("θ: " + std::to_string((int)m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).at(5)));
    //Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(1)));
    //DebugOutF("y: " + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(0)));
    //DebugOutF("θ: " + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).at(5)));

    //Store current posisition in a pose2d
    m_AbsolutePose = GetFieldPose();
    //Blue 
    //DebugOutF("Blue");
    // m_AbsolutePose = m_AbsolutePose.RelativeTo(kBlueOrigin);
  }
}

Pose2d Vision::GetFieldPose(){
  if(m_LimelightTable->GetNumber("tv", 0.0) == 1.0 && m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).size() != 0) {
    m_TempPose = 
      Pose2d(
        units::meter_t(m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).at(0)), 
        units::meter_t(m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).at(1)),
      Rotation2d(units::radian_t(Deg2Rad(m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).at(5))))
    );
    //DebugOutF(" tx: " + std::to_string(m_LimelightTable->GetNumber("tx", 0.0)));
    //DebugOutF("x: " + std::to_string(m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).at(0)));
    //DebugOutF("y: " + std::to_string(m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).at(1)));
    //DebugOutF("theta: " + std::to_string(m_LimelightTable->GetNumberArray("botpose_wpiblue", std::span<double>()).at(5)));
    //change area (when we are using two limelights; used to determine which april tag to use to orient depending on which one can see more april tag)
  }
  return m_TempPose;
}

//1 for front 0 for back
// std::string Vision::FrontBack(std::string key) {}

double Vision::DistanceFromAprilTag(double ID) {
  double x = IDMap[0][(int)ID - 1] - m_AbsolutePose.X().value();
  double y = IDMap[1][(int)ID - 1] - m_AbsolutePose.Y().value();
  DebugOutF("(x, y): (" + std::to_string(m_AbsolutePose.X().value()) + ", " + std::to_string(m_AbsolutePose.Y().value()) + ")");
  DebugOutF("id map x: " + std::to_string(IDMap[0][(int)ID - 1]));
  DebugOutF("id map y: " + std::to_string(IDMap[1][(int)ID - 1]));
  DebugOutF("deltax: " + std::to_string(x));
  DebugOutF("deltay: " + std::to_string(y));
  return sqrt(pow(x, 2) + pow(y, 2));
}

units::angle::radian_t Vision::VisionRobotYaw(double ID) {
  double x = IDMap[0][(int)ID - 1] - m_AbsolutePose.X().value();
  double y = IDMap[1][(int)ID - 1] - m_AbsolutePose.Y().value();
  // DebugOutF("inside vision robot yaw --> target yaw in radians: " + std::to_string(atan(x / y)));
  return units::angle::radian_t(atan(x / y));
}

units::angle::degree_t Vision::ShooterAngle(double ID) {
  double x = IDMap[0][(int)ID - 1] - m_AbsolutePose.X().value();
  double y = IDMap[1][(int)ID - 1] - m_AbsolutePose.Y().value();
  double distance = sqrt(pow(IDMap[0][(int)ID - 1], 2) + pow(IDMap[1][(int)ID - 1], 2));
  double height = IDMap[2][(int)ID - 1] - robotHeight;
}

double Vision::relativeDistancex(){  
    double targetOffsetAngle_Vertical = m_LimelightTable->GetNumber("ty",0.0);

    double limelightMountAngleDegrees = -1.4;//rough estimate

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 18.25; 

    // distance from the target to the floor
    double goalHeightInches = 23.80; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/tan(angleToGoalRadians);
    DebugOutF("delta x: " + std::to_string(distanceFromLimelightToGoalInches));
}

double Vision::GetIDMapValue(int coord, int id)
{
  return IDMap[coord][id - 1];
}
