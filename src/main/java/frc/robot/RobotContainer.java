// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.PIDChargeStationAutoBalCommand;
import frc.robot.commands.ElevatorManualDownCommand;
import frc.robot.commands.ElevatorManualUpCommand;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.TestAuto;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.io.ControlPanel;
import frc.robot.io.Dashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChargeStationAutoBalCommand;
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
    private final PhotonVisionSubsystem vision;

    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem drivetrain;

    private final Dashboard dashboard;
    private final ElevatorSubsystem elevator;
    
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveJoystick = new Joystick(0);
        turnJoystick = new Joystick(1);
        controlPanel = new ControlPanel(2);
        vision = new PhotonVisionSubsystem();
        drivetrain = new DrivetrainSubsystem();

        dashboard = Dashboard.getInstance();
        elevator = new ElevatorSubsystem();
        

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
        JoystickButton zeroGyroButton = new JoystickButton(controlPanel, 2);

        zeroGyroButton.onTrue(new InstantCommand(() -> drivetrain.zeroGyro(), drivetrain));

        /*
         * Charge station auto balancing button bindings
         */
        JoystickButton autoBalance = new JoystickButton(controlPanel, 9);
        JoystickButton simpleAutoBalance = new JoystickButton(controlPanel, 10);

        autoBalance.whileTrue(new PIDChargeStationAutoBalCommand(drivetrain));
        simpleAutoBalance.whileTrue(new ChargeStationAutoBalCommand(drivetrain, 1, -1, 3));

        /*
         * Elevator button bindings
         */

        JoystickButton elevatorCubeGroundPickUpButton = new JoystickButton(controlPanel, 8);
        JoystickButton elevatorConeGroundPickupButton = new JoystickButton(controlPanel, 2);
        JoystickButton elevatorBabyBirdButton = new JoystickButton(controlPanel, 4);
        JoystickButton elevatorMidScoreButton = new JoystickButton(controlPanel, 3);
        JoystickButton elevatorTopScoreButton = new JoystickButton(controlPanel, 5);

        elevatorCubeGroundPickUpButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.FLOORCUBE, elevator));
        elevatorConeGroundPickupButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.FLOORCONE, elevator));
        elevatorBabyBirdButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.BABYBIRD, elevator));
        elevatorMidScoreButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.MIDSCORE, elevator));
        elevatorTopScoreButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.TOPSCORE, elevator));

        // TODO: Update values
        JoystickButton manualElevatorUpButton = new JoystickButton(controlPanel, 12);
        JoystickButton manualElevatorDownButton = new JoystickButton(controlPanel, 11);
        manualElevatorUpButton.whileTrue(new ElevatorManualUpCommand(elevator));
        manualElevatorDownButton.whileTrue(new ElevatorManualDownCommand(elevator));
        
        JoystickButton resetElevatorEncoderButton = new JoystickButton(controlPanel, 4);
        resetElevatorEncoderButton.onTrue(new InstantCommand(() -> elevator.zeroEncoder(), elevator));

        /*
         * Intake button bindings
         */
        JoystickButton intakeButton = new JoystickButton(driveJoystick, 3);
        JoystickButton reverseIntakeButton = new JoystickButton(driveJoystick, 2);
    }
      

    public void zeroOdometry() {
        drivetrain.zeroGyro();
        drivetrain.zeroOdometry();
    }
    // ahhhhhhh
    /**
     * Use this to pass the autonomous commd to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new TestAuto(drivetrain);
    }
}
