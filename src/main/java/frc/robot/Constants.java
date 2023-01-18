// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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
    public static class Falcon500SwerveModule {
        public static final int TICKS_PER_ROTATION = 2048;
    }

    public static class SwerveModule {        
        /*
         * The maximum voltage that will be delivered to the drive motors.
         * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE_VOLTS = 12.0;
        public static final double DRIVE_CURRENT_LIMIT_AMPS = 80.0;
        public static final double STEER_CURRENT_LIMIT_AMPS = 20.0;

        public static final int CAN_TIMEOUT_MS = 250;

        public static final double WHEEL_DIAMETER_METERS = 0.1016;
        public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);
        public static final double STEER_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
        public static final int TICKS_PER_ROTATION = 2048;

        // Distance traveled in meters for each encoder tick adjusted for the module gear ratio
        public static final double DRIVE_METERS_PER_TICK = (Math.PI * WHEEL_DIAMETER_METERS /
            TICKS_PER_ROTATION) * DRIVE_REDUCTION;

        public static final double STEER_RADIANS_PER_TICK = (2.0 * Math.PI / 
            TICKS_PER_ROTATION) * STEER_REDUCTION;

        public static final double STEER_P = 0.2;
        public static final double STEER_I = 0.0;
        public static final double STEER_D = 0.1;

        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * By default this value is setup for a Mk3 standard module using Falcon500s to drive.
         * An example of this constant for a Mk4 L2 module with NEOs to drive is:
         * 5880.0 (RPM) / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            DRIVE_REDUCTION * WHEEL_DIAMETER_METERS * Math.PI;
        /*
         * The theoretical maximum angular velocity of the robot in radians per second.
         * This is a measure of how fast the robot can rotate in place.
         */
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(
            Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, 
            Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0
        );
    }

    public static class Drivetrain {
        // The left-to-right distance between the drivetrain wheels
        // Should be measured from center to center.
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.5);
        // The front-to-back distance between the drivetrain wheels.
        // Should be measured from center to center.
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.25);

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 5;
        public static final Rotation2d FRONT_LEFT_MODULE_STEER_OFFSET = new Rotation2d(2.477);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 9;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 8;
        public static final Rotation2d FRONT_RIGHT_MODULE_STEER_OFFSET = new Rotation2d(3.203);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2;
        public static final Rotation2d BACK_LEFT_MODULE_STEER_OFFSET = new Rotation2d(5.603);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 12;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
        public static final Rotation2d BACK_RIGHT_MODULE_STEER_OFFSET = new Rotation2d(5.217);
    }
}
