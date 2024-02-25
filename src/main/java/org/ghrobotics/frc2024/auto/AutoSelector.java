package org.ghrobotics.frc2024.auto;

import org.ghrobotics.frc2024.RobotState;
import org.ghrobotics.frc2024.subsystems.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSelector {
  private final SendableChooser<Command> auto_chooser_;
  
  private final Drive drive_;
  private final RobotState robot_state_;
  
  PathPlannerPath pickup_path = PathPlannerPath.fromPathFile("Pickup");
  PathPlannerPath shoot_path = PathPlannerPath.fromPathFile("Shoot");
  PathPlannerPath pickup_again_path = PathPlannerPath.fromPathFile("Pickup Again");
  PathPlannerPath shoot_again_path = PathPlannerPath.fromPathFile("Shoot Again");

  public AutoSelector(Drive drive, RobotState robot_state) {
    
    drive_ = drive;
    robot_state_ = robot_state;

    

    AutoBuilder.configureHolonomic(
      robot_state_::getPosition,
      robot_state_::reset,
      drive_::getSpeeds,
      drive_::setSpeeds,
      new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.kP, 0.0, 0.0),
        new PIDConstants(Constants.kP, 0.0, 0.0),
        3.0,
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

    auto_chooser_ = AutoBuilder.buildAutoChooser();


    SmartDashboard.putData("Auto Chooser", auto_chooser_);
  }

  public Command getAutonomousCommand() {
    Command shootPath = AutoBuilder.followPath(shoot_path);
    Command pickupPath = AutoBuilder.followPath(pickup_path);
    Command pickupAgainPath = AutoBuilder.followPath(pickup_again_path);
    Command shoootagainPath = AutoBuilder.followPath(shoot_again_path);
    
    return new SequentialCommandGroup(
      pickupPath,
      shootPath,
      pickupAgainPath,
      shoootagainPath
    );
  }

  public Pose2d getStartPosition(){
    return pickup_path.getStartingDifferentialPose();
  }

  public static class Constants {
    public static double kP = 4.5;
  }
}