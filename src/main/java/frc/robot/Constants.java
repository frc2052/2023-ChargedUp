// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    }
    
    public static final class Elevator {
        public static final int BELT_MOTOR = 15;

        public static final int LIMIT_SWITCH_DIO_CHANNEL = 0;

        public static final double BELT_MOTOR_P = 0.25;
        public static final double BELT_MOTOR_I = 0;
        public static final double BELT_MOTOR_D = 0;

        public static final double MANUAL_UP_SPEED = 0.15;
        public static final double MANUAL_DOWN_SPEED = -0.15;
        public static final double FEED_FORWARD = 0.05;

        private static final int FALCON500_TICKS_PER_ROTATION = 2048;
        public static final double BELT_MOTOR_CRUISE_VELOCITY = 5.0 * FALCON500_TICKS_PER_ROTATION;
        public static final double BELT_MOTOR_MAX_ACCELERATION = 5.0 * FALCON500_TICKS_PER_ROTATION;
        public static final int BELT_MOTOR_DEAD_ZONE_TICKS = 500;
    }

    public static final class Arm {
        public static final int ARM_SOLENOID_OUT_CHANNEL = 0;
        public static final int ARM_SOLENOID_IN_CHANNEL = 1;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR_ID = 13;

        public static final double INTAKE_IN_SPEED = 1.0;
        public static final double INTAKE_IN_SLOW_SPEED = 0.7;
        public static final double INTAKE_OUT_SPEED = -0.4;

        // Minimum allowable amps
        public static final double INTAKE_CRUISE_CURRENT_AMPS = 2.0;
        public static final double INTAKE_PEAK_CURRENT_THRESHOLD_AMPS = 10.0;
        public static final double INTAKE_PEAK_CURRENT_THRESHOLD_DURATION_SECONDS = 0.25;
    }
    
    public static final class AutoBalance {
        public static final double BALANCE_P = 0.01;
        public static final double BALANCE_I = 0.0;
        public static final double BALANCE_D = 0.0;
    
        public static final double BALANCE_TOLERANCE_DEGREES = 12;
        public static final double MAX_SPEED_METERS_PER_SECOND = 0.1;
    }

    public static final class Camera {
        public static final String CAMERA_NAME = "2052_Cicada";

        public static final int APRIL_TAG_PIPELINE = 1;
        public static final int REFLECTIVE_TAPE_PIPELINE = 0;

        public static final Transform3d CAMERA_POSITION_METERS = new Transform3d(
            new Translation3d(Units.inchesToMeters(6), Units.inchesToMeters(3.675), Units.inchesToMeters(41.75)), 
            new Rotation3d(0, Units.degreesToRadians(-5), 0)
        );

        public static final double APRIL_TAG_HEIGHT_METERS = Units.inchesToMeters(8);
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