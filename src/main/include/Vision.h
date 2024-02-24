#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"


using namespace frc;

class Vision {

    public:
        Vision();
        void PrintValues();
        void VisionInit();
        void CalcPose();
        void PushID();
        
        Pose2d GetPoseBlue();
        Pose2d GetPoseRed();
        Pose2d GetFieldPose();

        std::shared_ptr<nt::NetworkTable> GetLimeLight() { return nt::NetworkTableInstance::GetDefault().GetTable("limelight-front"); }

        std::string FrontBack(std::string key);
        units::angle::degree_t ShooterAngle(Pose2d pose, double ID);
        units::angle::degree_t VisionRobotYaw(Pose2d pose, double ID);

        const double IDMap[3][16] = {
        {593.68, 637.21, 652.73, 652.73, 578.77, 72.5, -1.5, -1.5, 14.02, 57.02, 468.69, 468.69, 441.74, 209.48, 182.73, 182.73},//x
        {9.68, 34.79, 196.17, 218.42, 323.00, 323.00, 218.42, 196.17, 34.79, 9.68, 146.19, 177.10, 161.62, 161.62, 177.10, 146.19},//y
        {53.38, 53.38, 57.13, 57.13, 53.38, 53.38, 57.13, 57.13, 53.38, 53.38, 52.00, 52.00, 52.00, 52.00, 52.00, 52.00}};//z (height)

        const double robotHeight = 0;

    private:
        double m_Area;
        std::shared_ptr<nt::NetworkTable> m_LimelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front");
        std::shared_ptr<nt::NetworkTable> m_FMS = nt::NetworkTableInstance::GetDefault().GetTable("FMSInfo");

        Pose2d m_AbsolutePose;
        Pose2d m_TempPose;

        const Pose2d kBlueOrigin = Pose2d(units::meter_t(-8.397494), units::meter_t(-3.978656), units::radian_t(0));
        const Pose2d kRedOrigin = Pose2d(units::meter_t(8.12816), units::meter_t(-4.00812), units::radian_t(0));
};