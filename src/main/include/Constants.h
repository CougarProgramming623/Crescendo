// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <math.h>

//#define ROBOT_WHEELBASE
// #define ROBOT_WHEELBASE
#define O12

#define DRIVE_REDUCTION ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))
#define STEER_REDUCTION ((14.0 /50.0) * (10.0 / 60.0))

#define STEER_ENCODER_POSITION_CONSTANT (2.0 * M_PI / 2048 * STEER_REDUCTION)
#define STEER_ENCODER_VELOCITY_CONSTANT (STEER_ENCODER_POSITION_CONSTANT * 10.0)

#define ENCODER_RESET_MAX_ANGULAR_VELOCITY (Deg2Rad(0.5))
#define ENCODER_RESET_ITERATIONS 500

#define WHEEL_DIAMETER 0.10033

#ifdef ROBOT_WHEELBASE

    #define DRIVETRAIN_TRACKWIDTH_METERS 0.5644

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    #define DRIVETRAIN_WHEELBASE_METERS 0.666

    #define FRONT_LEFT_MODULE_DRIVE_MOTOR 32
    #define FRONT_LEFT_MODULE_STEER_MOTOR 35
    #define FRONT_LEFT_MODULE_STEER_OFFSET -137
    #define FRONT_LEFT_MODULE_ENCODER_PORT 3

    #define FRONT_RIGHT_MODULE_DRIVE_MOTOR 42
    #define FRONT_RIGHT_MODULE_STEER_MOTOR 34
    #define FRONT_RIGHT_MODULE_STEER_OFFSET -84.67
    #define FRONT_RIGHT_MODULE_ENCODER_PORT 1

    #define BACK_LEFT_MODULE_DRIVE_MOTOR 41
    #define BACK_LEFT_MODULE_STEER_MOTOR 36
    #define BACK_LEFT_MODULE_STEER_OFFSET -140.6
    #define BACK_LEFT_MODULE_ENCODER_PORT 2

    #define BACK_RIGHT_MODULE_DRIVE_MOTOR 37
    #define BACK_RIGHT_MODULE_STEER_MOTOR 33
    #define BACK_RIGHT_MODULE_STEER_OFFSET -194.98
    #define BACK_RIGHT_MODULE_ENCODER_PORT 0


    #define LIMELIGHT_HEIGHT    38.57625   //cm
    //#define LIMELIGHT_HEIGHT 44.846875 //cm on C
    #define TARGET_HEIGHT_TALL  69    // Loading Zone //cm
    //#define TARGET_HEIGHT_SHORT 31.4   // Grid //cm
    #define TARGET_HEIGHT_SHORT 31.59125 //cm on the cone roughly 12.5 in
    #define LIMELIGHT_ANGLE  2.25     // degrees
#endif

#ifdef O12

    #define DRIVETRAIN_TRACKWIDTH_METERS 0.61595

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    #define DRIVETRAIN_WHEELBASE_METERS 0.61595

    #define FRONT_LEFT_MODULE_DRIVE_MOTOR 54
    #define FRONT_LEFT_MODULE_STEER_MOTOR 58
    #define FRONT_LEFT_MODULE_STEER_OFFSET -204.9//-204.7//-183.2//-203.6 //-207.8//JJ-180 //-198.2 //-207.8
    #define FRONT_LEFT_MODULE_ENCODER_PORT 2

    #define FRONT_RIGHT_MODULE_DRIVE_MOTOR 38
    #define FRONT_RIGHT_MODULE_STEER_MOTOR 61
    #define FRONT_RIGHT_MODULE_STEER_OFFSET -182.8//-181.2//-180.1//-181.9 //-178.47//JJ-179 //-180.97 //-178.47
    #define FRONT_RIGHT_MODULE_ENCODER_PORT 1

    #define BACK_LEFT_MODULE_DRIVE_MOTOR 52
    #define BACK_LEFT_MODULE_STEER_MOTOR 59
    #define BACK_LEFT_MODULE_STEER_OFFSET -208.4//-146.9//-238.0//-146.9//-200.9//-164.0 //-161.51//-178.25// -180.43
    #define BACK_LEFT_MODULE_ENCODER_PORT 3

    #define BACK_RIGHT_MODULE_DRIVE_MOTOR 51
    #define BACK_RIGHT_MODULE_STEER_MOTOR 60
    #define BACK_RIGHT_MODULE_STEER_OFFSET -293.4//-299.3//-176.5//-297.6 //-296.93//JJ-359//-294.17 //-296.93
    #define BACK_RIGHT_MODULE_ENCODER_PORT 0


    // #define LIMELIGHT_HEIGHT    -1   //cm
    // //#define LIMELIGHT_HEIGHT 44.846875 //cm on C
    // //#define TARGET_HEIGHT_SHORT 31.4   // Grid //cm
    // #define LIMELIGHT_ANGLE  -1     // degrees


#endif

#define TARGET_HEIGHT_TALL  69    // Loading Zone //cm
#define TARGET_HEIGHT_SHORT 31.59125 //cm on the cone roughly 12.5 in

#define DRIVE_ENCODER_POSITION_CONSTANT (M_PI * WHEEL_DIAMETER * DRIVE_REDUCTION / 2048)
#define DRIVE_ENCODER_VELOCITY_CONSTANT (DRIVE_ENCODER_POSITION_CONSTANT * 10)

#define ENCODER_VOLTAGE_TO_DEGREE (360/4.809)

#define COB_GET_ENTRY   Robot::GetRobot()->GetCOB().GetTable().GetEntry
#define GET_VISION      Robot::GetRobot()->GetVision()
    
#define COB_KEY_IS_RED   "/FMSInfo/IsRedAlliance"

#define COB_KEY_DISTANCE "/COB/distance"

#define COB_KEY_BOT_POSE_FRONT "/limelight-front/botpose" //FIX
#define COB_KEY_BOT_POSE_BACK "/limelight-back/botpose"

#define COB_KEY_BOT_POSE_BLUE_FRONT "/limelight-front/botpose_wpiblue"
#define COB_KEY_BOT_POSE_BLUE_BACK  "/limelight-back/botpose_wpiblue"

#define COB_KEY_TV_FRONT "/limelight-front/tv"
#define COB_KEY_TV_BACK "/limelight-back/tv"

#define COB_KEY_TX_FRONT "/limelight-front/tx"
#define COB_KEY_TX_BACK "/limelight-back/tx"

#define COB_KEY_TA_FRONT "/limelight-front/ta"
#define COB_KEY_TA_BACK "/limelight-back/ta"

#define COB_KEY_MATCHTIME "/COB/matchTime"

constexpr int kDriverControllerPort = 0; //uhhhh idk it just wont build without this line

#define ARM_SUBSYSTEM

#ifdef ARM_SUBSYSTEM

//motor and sensor IDs
#define WRIST_MOTOR 39
#define PIVOT_MOTOR 30
#define TOP_INTAKE_MOTOR -1 //check
#define BOTTOM_INTAKE_MOTOR 15
#define STRINGPOT_ANALOG_INPUT_ID 4
#define STRINGPOT 4 
#define PIVOT_CAN_ID 0

//setpoints
#define PIVOT_CAN_DIFFERENCE_BETWEEN_STARTING_AND_LEVEL -1 //check

//Math constants
#define PIVOT_GEAR_RATIO 160.0
#define PIVOT_TICKS_PER_DEGREE (PIVOT_GEAR_RATIO*2048.0/360.0)

#define WRIST_GEAR_RATIO 156.522
#define WRIST_TICKS_PER_DEGREE (WRIST_GEAR_RATIO*2048.0/360.0)

#define ARM_TOTAL_TICKS 185684
#define ARM_TOTAL_DEGREES 209.1
#define CANCODER_MIN 28.5
#define CANCODER_MAX 237.6
#define CANCODER_ZERO 126.3

#define WRIST_TOTAL_TICKS 265679.39932
#define WRIST_TOTAL_DEGREES (WRIST_TOTAL_TICKS/WRIST_GEAR_RATIO/2048)*360
#define STRINGPOT_TOTAL_RANGE 512.0
#define STRINGPOT_TOP 978 //1125.0
#define STRINGPOT_BOTTOM 1490 //1637.0
#define STRINGPOT_ZERO 1210 //1349.0
#define WRIST_DEGREES_PER_STRINGPOT_UNITS (WRIST_TOTAL_DEGREES/STRINGPOT_TOTAL_RANGE)

#define PIVOT_DFLT_VEL 8000 //8400 working value
#define PIVOT_DFLT_ACC 10000 //8000 working value
#define WRIST_DFLT_VEL 14000 //10000 working value
#define WRIST_DFLT_ACC 28000 //20000 working value

#define PIVOT_ACC_DIVISOR 3.5

//button IDs
//BUTTONBOARD 0
#define CONE_MODE 15 //left
#define CUBE_MODE 16 //right

#define PIVOT_CONTROL 1 
#define WRIST_CONTROL 0 

#define INTAKE_BUTTON 20
#define OUTTAKE_BUTTON 21

#define ARM_OVERRIDE 1 
#define ARM_OVERRIDE_2 2

#define BIG_RED 22

//BUTTONBOARD 2
#define GRID_TL 2
#define GRID_TC 7 
#define GRID_TR 12
#define GRID_ML 3 
#define GRID_MC 8 
#define GRID_MR 10
#define GRID_BL 4
#define GRID_BC 9
#define GRID_BR 14

#define GROUND_PICKUP_MODE 5 //red
#define TRANSIT_MODE 13 //green
#define PLACING_MODE 15 //yellow

#define LEFT_GRID 1 
#define CENTER_GRID 6 
#define RIGHT_GRID 11

#define LED_YELLOW 19
#define LED_PURPLE 18

#endif

#define TLPOSE frc::Pose2d(units::meter_t(2), units::meter_t(1.61), frc::Rotation2d(units::degree_t(0)))
#define TCPOSE frc::Pose2d(units::meter_t(2), units::meter_t(1.05), frc::Rotation2d(units::degree_t(0)))
#define TRPOSE frc::Pose2d(units::meter_t(2), units::meter_t(.49), frc::Rotation2d(units::degree_t(0)))

#define MLPOSE frc::Pose2d(units::meter_t(2), units::meter_t(1.61), frc::Rotation2d(units::degree_t(180)))
#define MCPOSE frc::Pose2d(units::meter_t(2), units::meter_t(1.072), frc::Rotation2d(units::degree_t(180)))
#define MRPOSE frc::Pose2d(units::meter_t(2), units::meter_t(.49), frc::Rotation2d(units::degree_t(180)))

#define BLPOSE frc::Pose2d(units::meter_t(2), units::meter_t(1.61), frc::Rotation2d(units::degree_t(180)))
#define BCPOSE frc::Pose2d(units::meter_t(2), units::meter_t(1.05), frc::Rotation2d(units::degree_t(180)))
#define BRPOSE frc::Pose2d(units::meter_t(2), units::meter_t(.49), frc::Rotation2d(units::degree_t(180)))