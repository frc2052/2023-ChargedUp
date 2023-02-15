// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS = 4.335 + (Math.PI / 2);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS = 4.858 + (Math.PI / 2);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 7;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET_RADIANS = 1.612 + (Math.PI / 2);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS = 1.179 + (Math.PI / 2);
    }

    public static final class Dashboard {
        public static final String FIELD_RELATIVE_KEY = "Field Relative";
        public static final boolean FIELD_RELATIVE_DEFAULT = true;

        public static final String ELEVATOR_POSITION_KEY = "Elevator position";
    }

    public static final class Elevator {
        public static final int BELT_MOTOR = 13;

        public static final double BELT_MOTOR_P = 0.2;
        public static final double BELT_MOTOR_I = 0;
        public static final double BELT_MOTOR_D = 0;

        private static final int FALCON500_TICKS_PER_ROTATION = 2048;
        public static final double BELT_MOTOR_CRUISE_VELOCITY = 8.0 * FALCON500_TICKS_PER_ROTATION;
        public static final double BELT_MOTOR_MAX_ACCELERATION = 8.0 * FALCON500_TICKS_PER_ROTATION;
        public static final int BELT_MOTOR_DEAD_ZONE_TICKS = 250;
    }

    public static final class Auto {
        public static final double ROBOT_LENGTH_METERS = 0;

        public static final double CHANNEL_WIDTH_METERS = Units.inchesToMeters(59.375);

        public static final double CHARGE_STATION_DEPTH = Units.inchesToMeters(76.125);

        public static final double COMMUNITY_WIDTH_METERS = Units.feetToMeters(18);
        public static final double COMMUNITY_DEPTH_METERS = Units.inchesToMeters(193.25);

        public static final double DISTANCE_GRID_TO_CHARGE_STATION_METERS = Units.inchesToMeters(60.5625);
        public static final double DISTANCE_GRID_TO_GAME_PIECES_METERS = Units.inchesToMeters(224);
        
        public static final double DISTANCE_BETWEEN_GAME_PIECES_METERS = Units.feetToMeters(4);
        public static final double DISTANCE_WALL_TO_GAME_PIECE_METERS = Units.inchesToMeters(36.25);
        public static final double FIELD_WIDTH = Units.feetToMeters(26.291667);
       
    }
    
    public static final class Intake {
        public static final int INTAKE_MOTOR_PWM_PORT = 0;
    }

    public static final class Arm {
        public static final int ARM_SOLENOID_FORWARD_CHANNEL = 0;
        public static final int ARM_SOLENOID_REVERSE_CHANNEL = 0;
    }
}
