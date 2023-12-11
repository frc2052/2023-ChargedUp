// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Drivetrain {
        // The left-to-right distance between the drivetrain wheels
        // Should be measured from one center of the wheel to the other.
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.1875);
        // The front-to-back distance between the drivetrain wheels.
        // Should be measured from one center of the wheel to the other.
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.5);

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 12;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS = 4.335 - (Math.PI / 2);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS = 4.858 - (Math.PI / 2);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 7;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET_RADIANS = 1.612 - (Math.PI / 2);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS = 1.179 - (Math.PI / 2);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );
    }
    
    public static final class Elevator {
        public static final int BELT_MOTOR = 15;

        public static final int LIMIT_SWITCH_DIO_CHANNEL = 0;

        public static final double BELT_MOTOR_F = 0.054;
        public static final double BELT_MOTOR_P = 0.2046;
        public static final double BELT_MOTOR_I = 0.002;
        public static final double BELT_MOTOR_D = 1.023;

        public static final double MANUAL_UP_SPEED = 0.15;
        public static final double MANUAL_DOWN_SPEED = -0.15;
        public static final double FEED_FORWARD = 0.065;

        public static final double BELT_MOTOR_CRUISE_VELOCITY = 14000;
        public static final double BELT_MOTOR_MAX_ACCELERATION = 24000;
        public static final int BELT_MOTOR_DEAD_ZONE_TICKS = 250;
    }

    public static final class Arm {
        public static final int ARM_SOLENOID_OUT_CHANNEL = 0;
        public static final int ARM_SOLENOID_IN_CHANNEL = 1;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR_ID = 13;

        public static final double FRONT_PIXY_MOUNT_OFFSET_PIXELS = 5;

        public static final double INTAKE_IN_SPEED = 1.0;
        public static final double INTAKE_IN_SLOW_SPEED = 0.6;
        public static final double INTAKE_OUT_CONE_SPEED = -0.5;
        public static final double INTAKE_OUT_CUBE_SPEED = -1.0;

        // Minimum allowable amps
        public static final double INTAKE_CRUISE_CURRENT_AMPS = 2.0;
        public static final double INTAKE_PEAK_CURRENT_THRESHOLD_AMPS = 9.5;
        public static final double INTAKE_PEAK_CURRENT_THRESHOLD_DURATION_SECONDS = 0.4;
    }
    
    public static final class Score {
        public static final double MIN_SCORE_TIME_SECONDS = 0.25;
    }

    public static final class AutoBalance {
        public static final double BALANCE_P = 0.01;
        public static final double BALANCE_I = 0.0;
        public static final double BALANCE_D = 0.0;
    
        public static final double BALANCE_TOLERANCE_DEGREES = 3.5;
        public static final double MAX_SPEED_METERS_PER_SECOND = 0.1;
    }

    public static final class Camera {
        public static final String CAMERA_NAME = "2052_Cicada";

        public static final int REFLECTIVE_TAPE_PIPELINE = 0;
        public static final int APRIL_TAG_PIPELINE = 1;

        public static final Transform3d CAMERA_POSITION_METERS = new Transform3d(
            new Translation3d(Units.inchesToMeters(6), Units.inchesToMeters(3.675), Units.inchesToMeters(41.75)), 
            new Rotation3d(0, Units.degreesToRadians(-5), 0)
        );
    }

    public static final class PiCamera1 {
        public static final double X_OFFSET_INCHES = 6;
        public static final double Y_OFFSET_INCHES = 6.925;
        public static final double Z_OFFSET_INCHES = 41.75;

        public static final double THETA_X_OFFSET_DEGREES = 0.0; // roll
        public static final double THETA_Y_OFFSET_DEGREES = 0.0; // pitch
        public static final double THETA_Z_OFFSET_DEGREES = 0.0; // yaw

        public static final Transform3d PI_CAMERA_POSITION_METERS = new Transform3d(
            new Translation3d(Units.inchesToMeters(X_OFFSET_INCHES), Units.inchesToMeters(Y_OFFSET_INCHES), Units.inchesToMeters(Z_OFFSET_INCHES)), 
            new Rotation3d(Units.degreesToRadians(THETA_X_OFFSET_DEGREES), Units.degreesToRadians(THETA_Y_OFFSET_DEGREES), Units.degreesToRadians(THETA_Z_OFFSET_DEGREES))
        );
    }

    public static final class PiCamera2 {
        // TODO: add offsets of second camera
        public static final double X_OFFSET_INCHES = 0;
        public static final double Y_OFFSET_INCHES = 0;
        public static final double Z_OFFSET_INCHES = 0;

        public static final double THETA_X_OFFSET_DEGREES = 0.0; // roll
        public static final double THETA_Y_OFFSET_DEGREES = 0.0; // pitch
        public static final double THETA_Z_OFFSET_DEGREES = 0.0; // yaw

        public static final Transform3d PI_CAMERA_POSITION_METERS = new Transform3d(
            new Translation3d(Units.inchesToMeters(X_OFFSET_INCHES), Units.inchesToMeters(Y_OFFSET_INCHES), Units.inchesToMeters(Z_OFFSET_INCHES)), 
            new Rotation3d(Units.degreesToRadians(THETA_X_OFFSET_DEGREES), Units.degreesToRadians(THETA_Y_OFFSET_DEGREES), Units.degreesToRadians(THETA_Z_OFFSET_DEGREES))
        );
    }

    public static final class Compressor {
        public static final int PNEUMATIC_HUB_ID = 14;
        public static final int COMPRESSOR_MIN_PRESSURE = 100;
        public static final int COMPRESSOR_MAX_PRESSURE = 120;
    }

    public static final class Dashboard {
        public static final String DRIVE_MODE_KEY = "Drive Mode";
        public static final String ELEVATOR_POSITION_KEY = "Elevator Position";
        public static final String ELEVATOR_LIMIT_SWITCH_KEY = "Elevator Limit Switch"; 
        public static final String INTAKE_CURRENT_KEY = "Intake Current";
        public static final String PRESSURE_KEY = "Pressure";
        public static final String CAMERA_CONNECTION_KEY = "Camera Connected";
        public static final String AUTO_COMPILED_KEY = "Auto Compiled";
        public static final String AUTO_DESCRIPTION_KEY = "Auto Description";
        public static final String CONE_OFFSET_KEY = "Cone Offset";
    }

    public static final class Auto {
        public static final double ROBOT_LENGTH_INCHES = 28.5;

        public static final double NODE_WIDTH_INCHES = 18.5;
        public static final double NODE_DIVIDER_WIDTH_INCHES = 3;

        public static final double CHANNEL_WIDTH_INCHES = 59.375;

        public static final double CHARGE_STATION_DEPTH_INCHES = 76.125;

        public static final double COMMUNITY_WIDTH_INCHES = 216; //SKD: Verified
        public static final double NEAR_COMMUNITY_DEPTH_INCHES = 132.375;  //SKD: 139 inches
        public static final double FAR_COMMUNITY_DEPTH_INCHES = 193.25;  //SKD: 139 inches
        public static final double LOADING_ZONE_WIDTH_INCHES = 99;

        public static final double DISTANCE_GRID_TO_CHARGE_STATION_INCHES = 60.5625;
        public static final double DISTANCE_GRID_TO_GAME_PIECES_INCHES = 224;
        
        public static final double DISTANCE_BETWEEN_GAME_PIECES_INCHES = 48;
        public static final double DISTANCE_WALL_TO_GAME_PIECE_INCHES = 36.25;
        public static final double FIELD_WIDTH_INCHES = 315.5;
    }

    public static final class LEDs {
        // Binary arduino code output bits
        public static final int CHANNEL_1_PIN = 1; // 2^0
        public static final int CHANNEL_2_PIN = 2; // 2^1
        public static final int CHANNEL_3_PIN = 3; // 2^2
        public static final int CHANNEL_4_PIN = 4; // 2^3
        public static final int CHANNEL_5_PIN = 5; // 2^4
        public static final int CHANNEL_6_PIN = 6; // 2^4
        public static final int CHANNEL_7_PIN = 7; // 2^4
        public static final int CHANNEL_8_PIN = 8; // 2^4
    }
}