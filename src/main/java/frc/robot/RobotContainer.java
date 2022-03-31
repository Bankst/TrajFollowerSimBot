package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;

public class RobotContainer {

  public final XboxController m_xboxCtrl = new XboxController(0);
  
  public final DriveSubsystem m_driveSubsys = new DriveSubsystem();

  public RobotContainer() {
    configureButtonBindings();

    var teleopDriveCommand = new RunCommand(
      () -> {
        boolean boost = m_xboxCtrl.getLeftStickButton();
        var xPow = -m_xboxCtrl.getLeftY() * (boost ? 1.0 : 0.7);
        m_driveSubsys.teleopArcadeDrive(
          xPow,
          m_xboxCtrl.getRightX()
        );
      },
      m_driveSubsys
    );

    CommandScheduler.getInstance().setDefaultCommand(m_driveSubsys, teleopDriveCommand);
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    // var startPose1 = new Pose2d(3, 1, new Rotation2d());
    // var endPose1 = new Pose2d(7, 4, new Rotation2d());
    // var trajConfig = new TrajectoryConfig(4.5, 4.5);
    // var traj1 = TrajectoryGenerator.generateTrajectory(startPose1, List.of(), endPose1, trajConfig);
    // var traj1cmd = m_driveSubsys.getRamseteCommand(traj1);


    // var startPose2 = new Pose2d(8.56, 5.39, Rotation2d.fromDegrees(68.75));
    // var midPose2_1 = new Pose2d(8.87, 7.71, Rotation2d.fromDegrees(18.43));
    // var midPose2_2 = new Pose2d()
    // var traj2 = 
    var threeBallTraj = PathPlanner.loadPath("3 Ball Auto", 2, 3);
    return m_driveSubsys.getRamseteCommand(threeBallTraj, true);
  }
}
