// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.PIDChargeStationAutoBalCommand;
import frc.robot.commands.TestAuto;
import frc.robot.io.ControlPanel;
import frc.robot.io.Dashboard;
import frc.robot.io.Dashboard.Autos;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.DynamicAutoConfiguration;
import frc.robot.auto.DynamicAutoFactory;
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

    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem drivetrain;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveJoystick = new Joystick(0);
        turnJoystick = new Joystick(1);
        controlPanel = new ControlPanel(2);

        drivetrain = new DrivetrainSubsystem();

        SmartDashboard.putBoolean("Field Centric", true);
        
        drivetrain.setDefaultCommand(
            new DefaultDriveCommand(
                // Forward velocity supplier
                () -> driveJoystick.getY(),
                // Sideways velocity supplier
                () -> driveJoystick.getX(),
                // Rotation velocity supplier
                () -> turnJoystick.getX(),
                () -> Dashboard.getInstance().isFieldRelative(), //SmartDashboard.getBoolean("Field Centric", true),
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
        JoystickButton zeroGyroButton = new JoystickButton(turnJoystick, 2);
        JoystickButton autoBalance = new JoystickButton(driveJoystick, 3);
        JoystickButton simpleAutoBalance = new JoystickButton(driveJoystick, 4);

        autoBalance.whileTrue(new PIDChargeStationAutoBalCommand(drivetrain));
        simpleAutoBalance.whileTrue(new ChargeStationAutoBalCommand(drivetrain, 1, -1, 3));
        zeroGyroButton.onTrue(new InstantCommand(() -> drivetrain.zeroGyro(), drivetrain));
    }

    public void zeroOdometry() {
        drivetrain.zeroGyro();
        drivetrain.zeroOdometry();
    }

    /**
     * Use this to pass the autonomous commd to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return new TestAuto(drivetrain);

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
