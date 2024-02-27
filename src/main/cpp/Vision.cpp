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
}


void Vision::CalcPose(){
  // DebugOutF("calc Pose");
  if(m_LimelightTable->GetNumber("tv", 0.0) == 1 && m_LimelightTable->GetNumberArray("botpose-wpiblue", std::span<double>()).size() != 0){

    //print size
    //DebugOutF("size: " + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_BOT_POSE).GetDoubleArray(std::span<double>()).size()));
    // DebugOutF("Tv: "   + std::to_string((int)Robot::GetRobot()->GetCOB().GetTable().GetEntry(COB_KEY_TV).GetInteger(0)));


    // //print x, y, theta
    DebugOutF("\nx: " + std::to_string((int)m_LimelightTable->GetNumberArray("botpose-wpiblue", std::span<double>()).at(1)));
    DebugOutF("y: " + std::to_string((int)m_LimelightTable->GetNumberArray("botpose-wpiblue", std::span<double>()).at(0)));
    DebugOutF("θ: " + std::to_string((int)m_LimelightTable->GetNumberArray("botpose-wpiblue", std::span<double>()).at(5)));
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
  if(m_LimelightTable->GetNumber("tv", 0.0) == 1 && m_LimelightTable->GetNumberArray("botpose-wpiblue", std::span<double>()).size() != 0) {
    m_TempPose = Pose2d(units::meter_t(m_LimelightTable->GetNumberArray("botpose-wpiblue", std::span<double>()).at(0)), 
      units::meter_t(m_LimelightTable->GetNumberArray("botpose-wpiblue", std::span<double>()).at(1)),
      Rotation2d(units::radian_t(m_LimelightTable->GetNumberArray("botpose-wpiblue", std::span<double>()).at(5)))
    );
    //change area (when we are using two limelights; used to determine which april tag to use to orient depending on which one can see more april tag)
  }
  // if(m_LimelightTable->GetNumber("tv", 0.0) == 1 && m_LimelightTable->GetNumberArray("botpose", std::span<double>()).size() != 0 && m_FMS->GetBoolean("IsRedAlliance", false)) {
  //   m_TempPose = Pose2d(units::meter_t(m_LimelightTable->GetNumberArray("botpose", std::span<double>()).at(0)), 
  //     units::meter_t(m_LimelightTable->GetNumberArray("botpose", std::span<double>()).at(1)),
  //     Rotation2d(units::radian_t(m_LimelightTable->GetNumberArray("botpose", std::span<double>()).at(5)))
  //   );
  //   //change area (when we are using two limelights; used to determine which april tag to use to orient depending on which one can see more april tag)
  // }
  return m_TempPose;
}

void Vision::PushID(){
  COB_GET_ENTRY("/COB/apriltagID").SetDouble(COB_GET_ENTRY(FrontBack("tid")).GetInteger(-1));
}

//1 for front 0 for back
std::string Vision::FrontBack(std::string key){
  
  if(COB_GET_ENTRY(COB_KEY_TV_FRONT).GetInteger(0) == 1 && COB_GET_ENTRY(COB_KEY_BOT_POSE_FRONT).GetDoubleArray(std::span<double>()).size() != 0){
    m_Area = COB_GET_ENTRY(COB_KEY_TA_FRONT).GetDouble(0);
  }
  
  if(COB_GET_ENTRY(COB_KEY_TV_BACK).GetInteger(0) == 1 && COB_GET_ENTRY(COB_KEY_BOT_POSE_BACK).GetDoubleArray(std::span<double>()).size() != 0 && COB_GET_ENTRY(COB_KEY_TA_BACK).GetDouble(0) > m_Area){
    return ("/limelight-back/" + key);
  } else {
    return ("/limelight-front/" + key);
  }//O12





  // if(COB_GET_ENTRY("/limelight/tv").GetInteger(0) == 1 && COB_GET_ENTRY("/limelight/botpose").GetDoubleArray(std::span<double>()).size() != 0){
  //   m_Area = COB_GET_ENTRY("/limelight/ta").GetDouble(0);
  // }
  
  // if(COB_GET_ENTRY("/limelight/tv").GetInteger(0) == 1 && COB_GET_ENTRY("/limelight/botpose").GetDoubleArray(std::span<double>()).size() != 0 && COB_GET_ENTRY("/limelight/ta").GetDouble(0) > m_Area){
  //   return ("/limelight/" + key);
  // } else {
  //   return ("/limelight/" + key);
  // } //Saber


}

units::angle::degree_t Vision::VisionRobotYaw(double ID) {
  double x = IDMap[0][(int)ID - 1] - m_AbsolutePose.X().value();
  double y = IDMap[1][(int)ID - 1] - m_AbsolutePose.Y().value();
  DebugOutF("x: " + std::to_string(x) + "and y: " + std::to_string(y));
  return units::angle::degree_t(atan(x / y));
}

units::angle::degree_t Vision::ShooterAngle(double ID) {
  double x = IDMap[0][(int)ID - 1] - m_AbsolutePose.X().value();
  double y = IDMap[1][(int)ID - 1] - m_AbsolutePose.Y().value();
  double distance = sqrt(pow(IDMap[0][(int)ID - 1], 2) + pow(IDMap[1][(int)ID - 1], 2));
  double height = IDMap[2][(int)ID - 1] - robotHeight;
}