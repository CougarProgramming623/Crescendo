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
        double relativeDistancex();

        Pose2d GetFieldPose();

        std::shared_ptr<nt::NetworkTable> GetLimeLight() { return nt::NetworkTableInstance::GetDefault().GetTable("limelight-front"); }

        std::string FrontBack(std::string key);
        void setPriority(double id);
        units::angle::radian_t VisionRobotYaw(double ID);
        units::angle::degree_t ShooterAngle(double ID);

        const double IDMap[3][16] = {
        {15.0832, 16.1845, 16.5816, 16.5816, 14.7035, 1.8425, -0.0381, -0.0381, 0.356308, 1.44881, 11.8987, 11.8987, 11.2134, 5.30729, 4.64034, 4.64034},//x
        {0.245872, 0.884066, 4.97842, 5.54507, 8.2042, 8.2042, 5.54507, 4.97842, 0.884066, 0.245872, 3.71533, 4.49894, 4.10677, 4.10677, 4.49894, 3.71533},//y
        {1.35535, 1.35535, 1.45070, 1.45070, 1.35535, 1.35535, 1.45070, 1.45070, 1.35535, 1.35535, 1.32080, 1.32080, 1.32080, 1.32080, 1.32080, 1.32080}};//z (height)

        const double robotHeight = 0;

    private:
        double m_Area;
        std::shared_ptr<nt::NetworkTable> m_LimelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight-front");
        std::shared_ptr<nt::NetworkTable> m_FMS = nt::NetworkTableInstance::GetDefault().GetTable("FMSInfo");

        Pose2d m_AbsolutePose;
        Pose2d m_TempPose;

        // const Pose2d kBlueOrigin = Pose2d(units::meter_t(-8.397494), units::meter_t(-3.978656), units::radian_t(0));
        // const Pose2d kRedOrigin = Pose2d(units::meter_t(8.12816), units::meter_t(-4.00812), units::radian_t(0));
};