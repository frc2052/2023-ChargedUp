// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.elevator.ElevatorManualDownCommand;
import frc.robot.commands.elevator.ElevatorManualUpCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.io.ControlPanel;
import frc.robot.io.Dashboard;
import frc.robot.io.Dashboard.DriveMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.DynamicAutoConfiguration;
import frc.robot.auto.DynamicAutoFactory;
import frc.robot.auto.RedLeftScoreOneBalanceAuto;
import frc.robot.auto.RedLeftScoreTwoBalanceAuto;

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
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private final ElevatorSubsystem elevator;
    //private final PhotonVisionSubsystem vision;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveJoystick = new Joystick(0);
        turnJoystick = new Joystick(1);
        controlPanel = new ControlPanel(2);

        drivetrain = new DrivetrainSubsystem();
        arm = new ArmSubsystem();
        intake = new IntakeSubsystem();
        elevator = new ElevatorSubsystem();
        //vision = new PhotonVisionSubsystem();

        new PneumaticsSubsystem();

        drivetrain.setDefaultCommand(
            new DefaultDriveCommand(
                // Forward velocity supplier
                () -> driveJoystick.getY(),
                // Sideways velocity supplier
                () -> driveJoystick.getX(),
                // Rotation velocity supplier
                () -> turnJoystick.getX(),
                () -> Dashboard.getInstance().getDriveMode() == DriveMode.FIELD_CENTRIC,
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

        JoystickButton autoBalance = new JoystickButton(controlPanel, 9);
        autoBalance.whileTrue(new ChargeStationBalanceCommand(drivetrain));

        // JoystickButton aprilTagDriveButton = new JoystickButton(driveJoystick, 1);
        // aprilTagDriveButton.whileTrue(new AprilTagDriveCommand(drivetrain, vision));

        /*
         * Elevator button bindings
         */
        Trigger elevatorStartingButton = new Trigger(() -> controlPanel.getY() < -0.5);
        elevatorStartingButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator));

        JoystickButton elevatorCubeGroundPickUpButton = new JoystickButton(controlPanel, 8);
        elevatorCubeGroundPickUpButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.FLOOR_CUBE, elevator));

        JoystickButton elevatorConeGroundPickupButton = new JoystickButton(controlPanel, 2);
        elevatorConeGroundPickupButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.FLOOR_CONE, elevator));

        JoystickButton elevatorBabyBirdButton = new JoystickButton(controlPanel, 4);
        elevatorBabyBirdButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator));

        JoystickButton elevatorMidScoreButton = new JoystickButton(controlPanel, 3);
        elevatorMidScoreButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.MID_SCORE, elevator));

        JoystickButton elevatorTopScoreButton = new JoystickButton(controlPanel, 5);
        elevatorTopScoreButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.TOP_SCORE, elevator));

        JoystickButton manualElevatorUpButton = new JoystickButton(controlPanel, 12);
        manualElevatorUpButton.whileTrue(new ElevatorManualUpCommand(elevator));

        JoystickButton manualElevatorDownButton = new JoystickButton(controlPanel, 11);
        manualElevatorDownButton.whileTrue(new ElevatorManualDownCommand(elevator));
        
        /*
         * Arm button bindings
         */
        JoystickButton driverIntakeArmToggle = new JoystickButton(driveJoystick, 1);
        JoystickButton controlPanelIntakeArmToggle = new JoystickButton(controlPanel, 1);
        driverIntakeArmToggle.or(controlPanelIntakeArmToggle).onTrue(new InstantCommand(() -> arm.toggleArm(), arm));

        JoystickButton armInButton = new JoystickButton(driveJoystick, 6);
        armInButton.onTrue(new InstantCommand(() -> arm.armIn(), arm));

        JoystickButton armOutButton = new JoystickButton(driveJoystick, 7);
        armOutButton.onTrue(new InstantCommand(() -> arm.armOut(), arm));

        /*
         * Intake button bindings
         */
        JoystickButton controlPanelIntakeInButton = new JoystickButton(controlPanel, 7);
        JoystickButton driverIntakeInButton = new JoystickButton(driveJoystick, 3);
        driverIntakeInButton.or(controlPanelIntakeInButton).whileTrue(new IntakeInCommand(intake));
        
        JoystickButton controlPanelIntakeOutButton = new JoystickButton(controlPanel, 6);
        JoystickButton driverIntakeOutButton = new JoystickButton(driveJoystick, 2);
        driverIntakeOutButton.or(controlPanelIntakeOutButton).whileTrue(new IntakeOutCommand(intake));
    }

    public void zeroOdometry() {
        drivetrain.zeroGyro();
        drivetrain.resetOdometry(new Pose2d());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        switch (Dashboard.getInstance().getAuto()) {
            case DYNAMIC_AUTO_FACTORY:
                return new DynamicAutoFactory(drivetrain, elevator, intake, arm).getAuto(
                    new DynamicAutoConfiguration(
                        Dashboard.getInstance().getGrid(), 
                        Dashboard.getInstance().getNode(),
                        Dashboard.getInstance().getExitChannel(),
                        Dashboard.getInstance().getGamePiece(), 
                        Dashboard.getInstance().getScoreGamePiece(), 
                        Dashboard.getInstance().getScoreGrid(), 
                        Dashboard.getInstance().getScoreNode(), 
                        Dashboard.getInstance().getEnterChannel(), 
                        false
                    )
                );
            
            case RED_LEFT_SCORE_ONE_BALANCE:
                //return new TestAuto(drivetrain, elevator, intake, arm);
                return new RedLeftScoreOneBalanceAuto(drivetrain, elevator, intake, arm);

                case RED_LEFT_SCORE_TWO_BALANCE:
                //return new TestAuto(drivetrain, elevator, intake, arm);
                return new RedLeftScoreTwoBalanceAuto(drivetrain, elevator, intake, arm);

            default:
                return null;
        }
    }
}
