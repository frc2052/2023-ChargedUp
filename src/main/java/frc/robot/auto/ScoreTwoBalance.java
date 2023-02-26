// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.auto;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants.Drivetrain;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ScoreTwoBalance extends AutoBase{

// /*Score gamepiece, move and rotate to gamepiece, move and rotate (strafe) to grid, 
// shoot gamepiece (w/o stopping), go to chargestation */

//   /** Creates a new scoretwoandbalence. */
//   public ScoreTwoBalance(DrivetrainSubsystem drivetrain,ElevatorSubsystem elevator,IntakeSubsystem intake,ArmSubsystem arm) {
//     super(drivetrain, elevator, intake, arm);

//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     Rotation2d rotation = Rotation2d.fromDegrees(0);
//     Pose2d endPose = new Pose2d(1, 0, Rotation2d.fromDegrees(0));
//     Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
//     SwerveControllerCommand commandOne = super.createSwerveCommand(startPose, endPose, rotation);
    
//     this.addCommands(commandOne);
//     //TODO; add score command

    

//   }  

  

//   @Override
//   public void init() {
//     // TODO Auto-generated method stub
    
//   }
// }
