// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.arm.ArmToggleCommand;
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
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.LEDSubsystem.LEDStatusMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.DynamicAutoConfiguration;
import frc.robot.auto.DynamicAutoFactory;

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
    private final LEDSubsystem leds;
    //private final PhotonVisionSubsystem vision;

    private final Compressor compressor;

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
        leds = LEDSubsystem.getInstance();
        //vision = new PhotonVisionSubsystem();

        compressor = new Compressor(Constants.Compressor.PNEUMATIC_HUB_ID, PneumaticsModuleType.REVPH);
        // Min and max recharge pressure, max pressure will stop at 115
        compressor.enableAnalog(100, 120);

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
        JoystickButton zeroGyroButton = new JoystickButton(controlPanel, 10);

        zeroGyroButton.onTrue(new InstantCommand(() -> drivetrain.zeroGyro(), drivetrain));

        /*
         * Charge station auto balancing button bindings
         */
        JoystickButton autoBalance = new JoystickButton(controlPanel, 9);

        autoBalance.whileTrue(new ChargeStationBalanceCommand(drivetrain));

        /*
         * LED button bindings
         */
        Trigger LEDOffButton = new Trigger(() -> controlPanel.getY() > 0.5);
        Trigger LEDConeButton = new Trigger(() -> controlPanel.getX() > 0.5);
        Trigger LEDCubeButton = new Trigger (() -> controlPanel.getX() < -0.5);

        LEDOffButton.onTrue(new InstantCommand(() -> leds.setLEDStatusMode(LEDStatusMode.OFF)));
        LEDConeButton.onTrue(new InstantCommand(() -> leds.setLEDStatusMode(LEDStatusMode.CONE)));
        LEDCubeButton.onTrue(new InstantCommand(() -> leds.setLEDStatusMode(LEDStatusMode.CUBE)));
        /*
         * Elevator button bindings
         */
        JoystickButton elevatorCubeGroundPickUpButton = new JoystickButton(controlPanel, 8);
        JoystickButton elevatorConeGroundPickupButton = new JoystickButton(controlPanel, 2);
        JoystickButton elevatorBabyBirdButton = new JoystickButton(controlPanel, 4);
        JoystickButton elevatorMidScoreButton = new JoystickButton(controlPanel, 3);
        JoystickButton elevatorTopScoreButton = new JoystickButton(controlPanel, 5);
        Trigger elevatorStartingButton = new Trigger(() -> controlPanel.getY() < -0.5);

        elevatorCubeGroundPickUpButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.FLOORCUBE, elevator));
        elevatorConeGroundPickupButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.FLOORCONE, elevator));
        elevatorBabyBirdButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.BABYBIRD, elevator));
        elevatorMidScoreButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.MIDSCORE, elevator));
        elevatorTopScoreButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.TOPSCORE, elevator));
        elevatorStartingButton.onTrue(new ElevatorPositionCommand(ElevatorPosition.STARTING, elevator));

        JoystickButton manualElevatorUpButton = new JoystickButton(controlPanel, 12);
        JoystickButton manualElevatorDownButton = new JoystickButton(controlPanel, 11);
        manualElevatorUpButton.whileTrue(new ElevatorManualUpCommand(elevator));
        manualElevatorDownButton.whileTrue(new ElevatorManualDownCommand(elevator));
        
        /*
         * Arm button bindings
         */
        JoystickButton intakeArmToggle = new JoystickButton(controlPanel, 1);
        intakeArmToggle.onTrue(new ArmToggleCommand(arm));

        /*
         * Intake button bindings
         */
        JoystickButton intakeInButton = new JoystickButton(controlPanel, 7);
        JoystickButton intakeOutButton = new JoystickButton(controlPanel, 6);

        intakeInButton.whileTrue(new IntakeInCommand(intake));
        intakeOutButton.whileTrue(new IntakeOutCommand(intake));
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
        //return new TestAuto(drivetrain);
        switch (Dashboard.getInstance().getAuto()) {
            case DYNAMIC_AUTO_FACTORY:
                return new DynamicAutoFactory(drivetrain).getAuto(
                    new DynamicAutoConfiguration(
                        Dashboard.getInstance().getGrid(), 
                        Dashboard.getInstance().getNode(),
                        Dashboard.getInstance().getChannel(),
                        Dashboard.getInstance().getGamePiece(), 
                        Dashboard.getInstance().getScoreGamePiece(), 
                        Dashboard.getInstance().getScoreGrid(), 
                        Dashboard.getInstance().getScoreNode(), 
                        Dashboard.getInstance().getEnterChannel(), 
                        false
                    )
                );
        
            default:
                return null;
        }
    }
}
