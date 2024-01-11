#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>


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

        std::string FrontBack(std::string key);

    private:
        double m_Area;

        Pose2d m_AbsolutePose;
        Pose2d m_TempPose;

        const Pose2d kBlueOrigin = Pose2d(units::meter_t(-8.397494), units::meter_t(-3.978656), units::radian_t(0)); 
        const Pose2d kRedOrigin = Pose2d(units::meter_t(8.12816), units::meter_t(-4.00812), units::radian_t(0));  

        



};