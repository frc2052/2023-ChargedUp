package frc.robot.auto.apriltagautos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoConfiguration;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.common.AutoTrajectoryConfig;
import frc.robot.commands.drive.ResetOdometryCommand;

@AutoDescription(description = "apriltag auto")
public class CenterOnTagAuto extends AutoBase{

    public CenterOnTagAuto(
        AutoConfiguration autoConfiguration, 
        AutoRequirements autoRequirements
        ) {
        super(autoConfiguration, autoRequirements);
    }

    @Override
    public void init() {
        final Pose2d initialPose = createPose2dInches(0, 0, 0);
        final Pose2d finalPose = createPose2dInches(24, 24, 0);

        final AutoTrajectoryConfig trajectory = new AutoTrajectoryConfig(1, 0.5, 1, 1, 1, 0, 0);

        addCommands(new ResetOdometryCommand(autoRequirements.getDrivetrain(), initialPose));
        
        SwerveControllerCommand path = createSwerveCommand(trajectory, initialPose, finalPose, createRotation(0));

        addCommands(path);
    }


    
}
