package org.ghrobotics.frc2024.auto;

import org.ghrobotics.frc2024.RobotState;
import org.ghrobotics.frc2024.Superstructure;
import org.ghrobotics.frc2024.commands.ArmPID;
import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Drive;
import org.ghrobotics.frc2024.subsystems.Feeder;
import org.ghrobotics.frc2024.subsystems.Intake;
import org.ghrobotics.frc2024.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutoSelector {
  // Subsystems
  private final Drive drive_;
  private final RobotState robot_state_;
  private final Superstructure superstructure_;
  private final Arm arm_;
  private final Intake intake_;
  private final Shooter shooter_;
  private final Feeder feeder_;

  // Different Paths
  PathPlannerPath middle_middle_intake_path = PathPlannerPath.fromPathFile("middle_middle_intake");
  PathPlannerPath middle_middle_shoot_path = PathPlannerPath.fromPathFile("middle_middle_shoot");
  PathPlannerPath middle_right_intake_path = PathPlannerPath.fromPathFile("middle_right_intake");

  Trigger trigger_ = new Trigger(() -> true);

  public AutoSelector(Drive drive, RobotState robot_state, Superstructure superstructure, Arm arm, Intake intake, Shooter shooter, Feeder feeder) {
    drive_ = drive;
    robot_state_ = robot_state;
    superstructure_ = superstructure;
    arm_ = arm;
    intake_ = intake;
    shooter_ = shooter;
    feeder_ = feeder;

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
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ArmPID(arm_, 20),
        new InstantCommand(() -> shooter_.setPercent(-0.75))
      ),
      new WaitCommand(1.0),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake_.setPercent(-0.5)),
        new InstantCommand(() -> feeder_.setPercent(0.5))
      ),
      new WaitCommand(1.0),
      new ParallelCommandGroup(
        new ArmPID(arm_, 2),
        new InstantCommand(() -> intake_.stopMotor()),
        new InstantCommand(() -> feeder_.stopMotor()),
        new InstantCommand(() -> shooter_.stopMotor())
      ),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake_.setPercent(-0.25)),
        AutoBuilder.followPath(middle_middle_intake_path)
      ),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake_.stopMotor()),
        new InstantCommand(() -> feeder_.stopMotor()),
        new InstantCommand(() -> shooter_.stopMotor())
      ),
      new ParallelCommandGroup(
        new ArmPID(arm_, 30),
        new InstantCommand(() -> shooter_.setPercent(-0.75))
      ),
      new WaitCommand(1.0),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake_.setPercent(-0.5)),
        new InstantCommand(() -> feeder_.setPercent(0.5))
      ),
      new WaitCommand(2.0),
      new ParallelCommandGroup(
        new ArmPID(arm_, 2),
        new InstantCommand(() -> intake_.stopMotor()),
        new InstantCommand(() -> feeder_.stopMotor()),
        new InstantCommand(() -> shooter_.stopMotor())
      )
    );
  }

  public Command justPath() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new InstantCommand(() -> intake_.setPercent(-0.25)),
        AutoBuilder.followPath(middle_right_intake_path)
      )
    );
  }

  // Get starting pose of autonomous path
  public Pose2d getStartingPose() {
    return middle_middle_intake_path.getStartingDifferentialPose();
  }
  

  public static class Constants {
    public static final double kP = 5.0;
  }
}
