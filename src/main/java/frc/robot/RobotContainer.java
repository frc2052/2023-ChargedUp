// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.TestAuto;
import frc.robot.io.ControlPanel;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Joystick driveJoystick;
    private final Joystick turnJoystick;
    private final ControlPanel controlPanel;

    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem drivetrain;
    private final ElevatorSubsystem elevator;
    private final IntakeSubsystem intake;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveJoystick = new Joystick(0);
        turnJoystick = new Joystick(1);
        controlPanel = new ControlPanel(2);

        drivetrain = new DrivetrainSubsystem();
        elevator = new ElevatorSubsystem();
        intake = new IntakeSubsystem();

        drivetrain.setDefaultCommand(
            new DefaultDriveCommand(
                // Forward velocity supplier
                () -> driveJoystick.getY(),
                // Sideways velocity supplier
                () -> driveJoystick.getX(),
                // Rotation velocity supplier
                () -> turnJoystick.getX(),
                () -> Dashboard.getInstance().isFieldRelative(),
                drivetrain
            )
        );
        elevator.setDefaultCommand(new RunCommand(() -> elevator.stop(), elevator));
        intake.setDefaultCommand(new RunCommand(() -> intake.stop(), intake));

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with a
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joystick}.
     */
    private void configureBindings() {
        /*
         * Drivetrain button bindings
         */
        JoystickButton zeroGyroButton = new JoystickButton(turnJoystick, 2);
        zeroGyroButton.onTrue(new InstantCommand(() -> drivetrain.zeroGyro(), drivetrain));

        /*
         * Elevator button bindings
         */
        JoystickButton elevatorGroundPickUpButton = new JoystickButton(controlPanel, 5);
        JoystickButton elevatorBottomRowButton = new JoystickButton(controlPanel, 3);
        // JoystickButton elevatorMiddleRowButton = new JoystickButton(controlPanel, 1);
        // JoystickButton elevatorTopRowButton = new JoystickButton(controlPanel, 1);
        elevatorGroundPickUpButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.TOP, elevator));
        elevatorBottomRowButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.BOTTOM, elevator));
        // elevatorMiddleRowButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.MIDDLE_ROW, elevator));
        // elevatorTopRowButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.TOP_ROW, elevator));

        // TODO: Update values
        JoystickButton manualElevatorUpButton = new JoystickButton(controlPanel, 1);
        JoystickButton manualElevatorDownButton = new JoystickButton(controlPanel, 1);
        manualElevatorUpButton.whileTrue(new RunCommand(() -> elevator.manualUp(), elevator));
        manualElevatorDownButton.whileTrue(new RunCommand(() -> elevator.manualDown(), elevator));
        
        JoystickButton resetElevatorEncoderButton = new JoystickButton(controlPanel, 4);
        resetElevatorEncoderButton.onTrue(new InstantCommand(() -> elevator.zeroEncoder(), elevator));

        /*
         * Intake button bindings
         */
        JoystickButton intakeButton = new JoystickButton(driveJoystick, 3);
        JoystickButton reverseIntakeButton = new JoystickButton(driveJoystick, 2);
        intakeButton.whileTrue(new RunCommand(() -> intake.intake(), intake));
        reverseIntakeButton.whileTrue(new RunCommand(() -> intake.reverseIntake(), intake));
    }

    public void zeroOdometry() {
        drivetrain.zeroGyro();
        drivetrain.zeroOdometry();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new TestAuto(drivetrain);
    }
}
