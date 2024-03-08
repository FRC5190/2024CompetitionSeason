package org.ghrobotics.frc2024.auto;

import org.ghrobotics.frc2024.RobotState;
import org.ghrobotics.frc2024.Superstructure;
import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutoSelector {
  // Subsystems
  private final Drive drive_;
  private final RobotState robot_state_;
  private final Superstructure superstructure_;
  private final Arm arm_;

  Trigger trigger_ = new Trigger(() -> true);

  public AutoSelector(Drive drive, RobotState robot_state, Superstructure superstructure, Arm arm) {
    drive_ = drive;
    robot_state_ = robot_state;
    superstructure_ = superstructure;
    arm_ = arm;

    // Autobuilder for Pathplanner
    AutoBuilder.configureHolonomic(
      robot_state_::getPosition,
      robot_state_::reset,
      drive_::getSpeeds,
      drive_::setsOpenSpeeds,
      new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.kP, 0.0, 0.0),
        new PIDConstants(Constants.kP, 0.0, 0.0),
        1.0,
        0.375,
        new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      drive_
    );
  }

  public Command followPath() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Straight");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("New Path");
    PathPlannerPath path3 = PathPlannerPath.fromPathFile("newpickup");

    return new SequentialCommandGroup(
      new InstantCommand(() -> trigger_.whileTrue(superstructure_.setShooter(-0.75))).withTimeout(3.0),
      AutoBuilder.followPath(path).withTimeout(3.0),
      new InstantCommand(() -> trigger_.whileTrue(superstructure_.setIntake(-0.5))).withTimeout(3.0),
      AutoBuilder.followPath(path2).withTimeout(3.0),
      AutoBuilder.followPath(path3).withTimeout(4.5)
    );
  }

  // Get starting pose of autonomous path
  public Pose2d getStartingPose() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Straight");
    return path.getStartingDifferentialPose();
  }
  

  public static class Constants {
    public static final double kP = 0.5;
  }
}
