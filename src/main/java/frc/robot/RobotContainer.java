// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeStopCommand;
import frc.robot.commands.score.MidScoreCommand;
import frc.robot.commands.score.ScoreCommand;
import frc.robot.commands.score.TopScoreCommand;
import frc.robot.auto.DynamicAutoFactory;
import frc.robot.auto.red.MiddleScoreOneExitBalance;
import frc.robot.auto.red.RedLeftScoreTwoBalanceAuto;
import frc.robot.auto.red.RedMiddleScoreOneBalance;
import frc.robot.auto.red.RedScoreOneBalanceAuto;
import frc.robot.commands.drive.ChargeStationBalanceCommand;
import frc.robot.commands.drive.AprilTagAlignCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.elevator.ElevatorManualDownCommand;
import frc.robot.commands.elevator.ElevatorManualUpCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.io.ControlPanel;
import frc.robot.io.Dashboard;
import frc.robot.io.Dashboard.DriveMode;
import frc.robot.io.Dashboard.Grid;
import frc.robot.io.Dashboard.Node;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import edu.wpi.first.math.geometry.Pose2d;
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
    
    private final DrivetrainSubsystem drivetrain;
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private final ElevatorSubsystem elevator;
    private final PhotonVisionSubsystem vision;

    private final DynamicAutoFactory autoFactory;

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
        vision = new PhotonVisionSubsystem();

        new PneumaticsSubsystem();

        autoFactory = new DynamicAutoFactory(drivetrain, elevator, intake, arm, vision);

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

        Trigger autoBalance = new Trigger(() -> controlPanel.getY() > 0.5);
        autoBalance.whileTrue(new ChargeStationBalanceCommand(drivetrain));

        // JoystickButton aprilTagDriveButton = new JoystickButton(turnJoystick, 1);
        // JoystickButton aprilTagLeftNodeDriveButton = new JoystickButton(turnJoystick, 4);
        // JoystickButton aprilTagRightNodeDriveButton = new JoystickButton(turnJoystick, 5);
        // aprilTagDriveButton.and(aprilTagLeftNodeDriveButton).whileTrue(
        //     new AprilTagAlignCommand(Node.LEFT_CONE, drivetrain, vision)
        // );
        // aprilTagDriveButton.and(aprilTagRightNodeDriveButton).whileTrue(
        //     new AprilTagAlignCommand(Node.RIGHT_CONE, drivetrain, vision)
        // );
        // aprilTagDriveButton.whileTrue(
        //     new AprilTagAlignCommand(Node.MIDDLE_CUBE, drivetrain, vision)
        // );

        JoystickButton xWheelsButton = new JoystickButton(controlPanel, 2);
        xWheelsButton.whileTrue(new RunCommand(drivetrain::xWheels, drivetrain));

        /*
         * LED button bindings
         */
        JoystickButton LEDOffButton = new JoystickButton(controlPanel, 9);
        JoystickButton LEDConeButton = new JoystickButton(controlPanel, 1);
        JoystickButton LEDCubeButton = new JoystickButton (controlPanel, 6);

        LEDOffButton.onTrue(new InstantCommand(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.OFF)));
        LEDConeButton.onTrue(new InstantCommand(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CONE)));
        LEDCubeButton.onTrue(new InstantCommand(() -> LEDSubsystem.getInstance().setLEDStatusMode(LEDStatusMode.CUBE)));

        /*
         * Elevator button bindings
         */
        Trigger elevatorStartingButton = new Trigger(() -> controlPanel.getX() < -0.5);
        JoystickButton elevatorCubeGroundPickUpButton = new JoystickButton(controlPanel, 12);
        JoystickButton elevatorConeGroundPickupButton = new JoystickButton(controlPanel, 10);
        JoystickButton elevatorBabyBirdButton = new JoystickButton(controlPanel, 5);
        
        elevatorStartingButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator));
        elevatorCubeGroundPickUpButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.FLOOR_CUBE, elevator));
        elevatorConeGroundPickupButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.FLOOR_CONE, elevator));
        elevatorBabyBirdButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.BABY_BIRD, elevator));

        JoystickButton manualElevatorUpButton = new JoystickButton(controlPanel, 3);
        JoystickButton manualElevatorDownButton = new JoystickButton(controlPanel, 4);

        manualElevatorUpButton.whileTrue(new ElevatorManualUpCommand(elevator));
        manualElevatorDownButton.whileTrue(new ElevatorManualDownCommand(elevator));

        /*
         * Score button bindings
         */
        JoystickButton elevatorMidScoreButton = new JoystickButton(controlPanel, 7);
        JoystickButton elevatorTopScoreButton = new JoystickButton(controlPanel, 8);
        JoystickButton scoreButton = new JoystickButton(driveJoystick, 1);
        
        scoreButton.whileTrue(new ScoreCommand(intake, arm, elevator));
        elevatorMidScoreButton.onTrue(new MidScoreCommand(elevator, arm));
        elevatorTopScoreButton.onTrue(new TopScoreCommand(elevator, arm));

        /*
         * Arm button bindings
         */
        JoystickButton controlPanelIntakeArmToggle = new JoystickButton(controlPanel, 11);
        JoystickButton driverIntakeArmToggle = new JoystickButton(driveJoystick, 6);

        driverIntakeArmToggle.or(controlPanelIntakeArmToggle).onTrue(new InstantCommand(() -> arm.toggleArm(), arm));

        /*
         * Intake button bindings
         */
        Trigger controlPanelIntakeInButton = new Trigger(() -> controlPanel.getX() > 0.5);
        JoystickButton driverIntakeInButton = new JoystickButton(driveJoystick, 3);

        driverIntakeInButton.or(controlPanelIntakeInButton).whileTrue(new IntakeInCommand(arm::isArmOut, intake));
        driverIntakeInButton.or(controlPanelIntakeInButton).onFalse(new IntakeStopCommand(intake));
        
        Trigger controlPanelIntakeOutButton = new Trigger(() -> controlPanel.getY() < -0.5);
        JoystickButton driverIntakeOutButton = new JoystickButton(driveJoystick, 2);

        driverIntakeOutButton.or(controlPanelIntakeOutButton).whileTrue(new IntakeOutCommand(intake));
        driverIntakeOutButton.or(controlPanelIntakeOutButton).onFalse(new IntakeStopCommand(intake));
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
            case SCORE_ONE_BALANCE:
                return new RedScoreOneBalanceAuto(
                    Dashboard.getInstance().getStartingGrid(), 
                    Dashboard.getInstance().getStartingNode(), 
                    Dashboard.getInstance().endChargeStation(),
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );

            case SCORE_TWO_BALANCE:
                return new RedLeftScoreTwoBalanceAuto(
                    Dashboard.getInstance().getStartingGrid(), 
                    Dashboard.getInstance().getStartingNode(),
                    Dashboard.getInstance().endChargeStation(),
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );

            case MIDDLE_SCORE_ONE_EXIT:
                return new MiddleScoreOneExitBalance(
                    Grid.MIDDLE_GRID, 
                    Dashboard.getInstance().getStartingNode(),
                    Dashboard.getInstance().endChargeStation(),
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );

            case MIDDLE_SCORE_ONE_BALANCE:
                return new RedMiddleScoreOneBalance(
                    Grid.MIDDLE_GRID,
                    Dashboard.getInstance().getStartingNode(),
                    Dashboard.getInstance().endChargeStation(),
                    drivetrain, 
                    elevator, 
                    intake, 
                    arm
                );

            case NO_AUTO:
            default:
                return null;

            //  case DYNAMIC_AUTO_FACTORY:
            //     return autoFactory.getAuto(
            //         new DynamicAutoConfiguration(
            //             Dashboard.getInstance().getStartingGrid(), 
            //             Dashboard.getInstance().getStartingNode(),
            //             Dashboard.getInstance().getExitChannel(),
            //             Dashboard.getInstance().getGamePiece(), 
            //             Dashboard.getInstance().scoreGamePiece(), 
            //             Dashboard.getInstance().getScoreGrid(), 
            //             Dashboard.getInstance().getScoreNode(),
            //             Dashboard.getInstance().endChargeStation()
            //         )
            //     );
            
            // case TEST_LEFT_SCORE_ONE_BALANCE:
            //     return autoFactory.getAuto(
            //         new DynamicAutoConfiguration(
            //             Grid.LEFT_GRID, 
            //             Dashboard.getInstance().getStartingNode(),
            //             Channel.LEFT_CHANNEL,
            //             GamePiece.FAR_LEFT_GAME_PIECE, 
            //             false, 
            //             null, 
            //             null,
            //             Dashboard.getInstance().endChargeStation()
            //         )
            //     );
        }
    }
}
