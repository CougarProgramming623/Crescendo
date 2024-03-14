// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <math.h>

// #define ROBOT_WHEELBASE
#define O12
// #define ANATOLI

#define DRIVE_REDUCTION ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))
#define STEER_REDUCTION ((14.0 /50.0) * (10.0 / 60.0))

#define STEER_ENCODER_POSITION_CONSTANT (2.0 * M_PI) // * STEER_REDUCTION
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

    #define FRONT_LEFT_MODULE_DRIVE_MOTOR 54 //inverted
    #define FRONT_LEFT_MODULE_STEER_MOTOR 58
    #define FRONT_LEFT_MODULE_STEER_OFFSET 4.27
    #define FRONT_LEFT_MODULE_ENCODER_PORT 2

    #define FRONT_RIGHT_MODULE_DRIVE_MOTOR 38
    #define FRONT_RIGHT_MODULE_STEER_MOTOR 61 //inverted
    #define FRONT_RIGHT_MODULE_STEER_OFFSET 3.67
    #define FRONT_RIGHT_MODULE_ENCODER_PORT 1

    #define BACK_LEFT_MODULE_DRIVE_MOTOR 32
    #define BACK_LEFT_MODULE_STEER_MOTOR 35 //inverted
    #define BACK_LEFT_MODULE_STEER_OFFSET 3.42
    #define BACK_LEFT_MODULE_ENCODER_PORT 3

    #define BACK_RIGHT_MODULE_DRIVE_MOTOR 51 //inverted
    #define BACK_RIGHT_MODULE_STEER_MOTOR 60
    #define BACK_RIGHT_MODULE_STEER_OFFSET 2.47
    #define BACK_RIGHT_MODULE_ENCODER_PORT 0

    #define MAX_VOLTAGE_WHEN_OFFSET 4.76

    #define CLIMB_MOTOR 31


    // #define LIMELIGHT_HEIGHT    -1   //cm
    // //#define LIMELIGHT_HEIGHT 44.846875 //cm on C
    // //#define TARGET_HEIGHT_SHORT 31.4   // Grid //cm
    // #define LIMELIGHT_ANGLE  -1     // degrees

#endif


#ifdef ANATOLI
    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    #define DRIVETRAIN_WHEELBASE_METERS 0.61595
    #define DRIVETRAIN_TRACKWIDTH_METERS 0.61595

    #define FRONT_LEFT_MODULE_DRIVE_MOTOR 41
    #define FRONT_LEFT_MODULE_STEER_MOTOR 36
    #define FRONT_LEFT_MODULE_STEER_OFFSET 0//4.27
    #define FRONT_LEFT_MODULE_ENCODER_PORT 2

    #define FRONT_RIGHT_MODULE_DRIVE_MOTOR 42
    #define FRONT_RIGHT_MODULE_STEER_MOTOR 34
    #define FRONT_RIGHT_MODULE_STEER_OFFSET 0//3.67
    #define FRONT_RIGHT_MODULE_ENCODER_PORT 1

    #define BACK_LEFT_MODULE_DRIVE_MOTOR 37
    #define BACK_LEFT_MODULE_STEER_MOTOR 52
    #define BACK_LEFT_MODULE_STEER_OFFSET 0//3.42
    #define BACK_LEFT_MODULE_ENCODER_PORT 3

    #define BACK_RIGHT_MODULE_DRIVE_MOTOR 53
    #define BACK_RIGHT_MODULE_STEER_MOTOR 59
    #define BACK_RIGHT_MODULE_STEER_OFFSET 0//2.47
    #define BACK_RIGHT_MODULE_ENCODER_PORT 0

    #define MAX_VOLTAGE_WHEN_OFFSET 0//4.76

    #define CLIMB_MOTOR 31

#endif


#define DRIVE_ENCODER_POSITION_CONSTANT (M_PI * WHEEL_DIAMETER/2) //DRIVE_REDUCTION
#define DRIVE_ENCODER_VELOCITY_CONSTANT (DRIVE_ENCODER_POSITION_CONSTANT * 10)

// #define ENCODER_VOLTAGE_TO_DEGREE (360/4.809)

constexpr int kDriverControllerPort = 0; //uhhhh idk it just wont build without this line

#define ARM_SUBSYSTEM

#ifdef ARM_SUBSYSTEM

    //motor and sensor IDs
    #define PIVOT_MOTOR 39
    #define SHOOTER1_MOTOR 30
    #define SHOOTER2_MOTOR 33
    #define FEEDER_MOTOR 22

    //#define TOP_INTAKE_MOTOR -1 //check
    //#define BOTTOM_INTAKE_MOTOR 15
    // #define STRINGPOT_ANALOG_INPUT_ID 4
    #define STRINGPOT_ANALOG_INPUT_ID 0
    //#define PIVOT_CAN_ID 0

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

    #define PIVOT_GEAR_RATIO 50
    #define PIVOT_TOTAL_ROTATIONS 4.98
    #define PIVOT_TOTAL_DEGREES ((PIVOT_TOTAL_ROTATIONS/PIVOT_GEAR_RATIO * 360) + STRINGPOT_ZERO_DEGREES)

    #define STRINGPOT_ZERO -1
    #define STRINGPOT_TOP -1
    #define STRINGPOT_TOTAL_RANGE -1
    #define STRINGPOT_ZERO_DEGREES 37

    #define PIVOT_ROTATIONS_PER_STRINGPOT_UNITS (PIVOT_TOTAL_ROTATIONS/STRINGPOT_TOTAL_RANGE)

    #define CLOSEUPSHOOTSTRINGPOT 555
    #define PICKUPSTRINGPOT 420

    #define PIVOT_DFLT_VEL 0//8000 - 2023 //8400 working value
    #define PIVOT_DFLT_ACC 0//10000 - 2023 //8000 working value
    #define WRIST_DFLT_VEL 0//14000 - 2023 //10000 working value
    #define WRIST_DFLT_ACC 0//28000 - 2023 //20000 working value

    #define PIVOT_ACC_DIVISOR 3.5

#endif

//button IDs
#define DUSTPANUP_LIMIT 0 //x axis
#define SHOOTER_SPEED 1 //y axis
#define FLYWHEEL_SWITCH 1
#define INTAKE_SWITCH 2
#define AIM_BUTTON 3
#define SHOOT_BUTTON 4
#define AMP_BUTTON 5
#define STOW_BUTTON 6
#define ALLUP_BUTTON 7
#define ARM_OVERRIDE 16
#define SHOOTER_DOWN 17
#define SHOOTER_UP 18
#define CLIMB_UP 19
#define CLIMB_DOWN 20
#define DUSTPAN_UP 21
#define DUSTPAN_DOWN 22