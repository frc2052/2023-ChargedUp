package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.Dashboard;

public class AprilTagSubsystem extends SubsystemBase {
  private Dashboard dashboard;


  public AprilTagSubsystem() {
    dashboard = Dashboard.getInstance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int tagNum = dashboard.getGoalTag();
  }
}
