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
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  // Sendable chooser
  private final SendableChooser<Command> routine_chooser_;

  private final SendableChooser<Pose2d> starting_position_chooser_;

  // Different Paths
  PathPlannerPath middle_middle_intake_path = PathPlannerPath.fromPathFile("middle_middle_intake");
  PathPlannerPath middle_middle_shoot_path = PathPlannerPath.fromPathFile("middle_middle_shoot");
  PathPlannerPath middle_right_intake_path = PathPlannerPath.fromPathFile("middle_right_intake");
  PathPlannerPath middle_left_intake_path = PathPlannerPath.fromPathFile("middle_left_intake");
  PathPlannerPath left_shoot_path = PathPlannerPath.fromPathFile("left_shoot");

  PathPlannerPath go_straight_path = PathPlannerPath.fromPathFile("go_straight");

  // Helper Commands
  Command stop_all_motor;

  // Four Note Auto Path
  PathPlannerPath left_intake_path = PathPlannerPath.fromPathFile("left_intake");
  PathPlannerPath middle_intake_path = PathPlannerPath.fromPathFile("middle_intake");
  PathPlannerPath right_intake_path = PathPlannerPath.fromPathFile("right_intake");

  PathPlannerPath left_one_intake_close_path = PathPlannerPath.fromPathFile("left_one_intake");

  // PathPlannerPath left_one_intake_path = PathPlannerPath.fromPathFile("left_one_intake");

  Command autonomous_command_;

  Trigger trigger_ = new Trigger(() -> true);

  public AutoSelector(Drive drive, RobotState robot_state, Superstructure superstructure, Arm arm, Intake intake, Shooter shooter, Feeder feeder) {
    drive_ = drive;
    robot_state_ = robot_state;
    superstructure_ = superstructure;
    arm_ = arm;
    intake_ = intake;
    shooter_ = shooter;
    feeder_ = feeder;

    
    

    stop_all_motor = new ParallelCommandGroup(
      new InstantCommand(() -> intake_.stopMotor()),
      new InstantCommand(() -> feeder_.stopMotor()),
      new InstantCommand(() -> shooter_.stopMotor()));
    

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
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        //   return alliance.get() == DriverStation.Alliance.Red;
        // }
        return false;
      },
      drive_
    );
    
    starting_position_chooser_ = new SendableChooser<Pose2d>();

    routine_chooser_ = AutoBuilder.buildAutoChooser();
    routine_chooser_.setDefaultOption("One Shot Taxi Straight", shootTaxi());
    routine_chooser_.addOption("Four note Auto", fourNoteFull());
    routine_chooser_.addOption("Three Note Left Auto", threeNoteMiddleAuto());

    starting_position_chooser_.setDefaultOption("One Shot Taxi Straight pose", getOneShotTaxiPose2d());
    starting_position_chooser_.addOption("Four Note Auto Pose", getFourNotePose2d());
    starting_position_chooser_.addOption("Three Note Left Auto Pose", getThreeNoteMiddlePose2d());
  }


  // public Command leftTwoNoteAuto() {
  //   return new SequentialCommandGroup(
      
  //   )
  // }

  public SendableChooser<Command> getRoutineChooser() {
    return routine_chooser_;
  }

  public SendableChooser<Pose2d> getPositionChooser() {
    return starting_position_chooser_;
  }

  public Command getSelectedRoutine() {
    return routine_chooser_.getSelected();
  }

  public Pose2d getSelectedPose2d() {
    return starting_position_chooser_.getSelected();
  }

  public Command fourNoteFull() {
    return new SequentialCommandGroup(
      fourNotePt1().withTimeout(5.0),
      fourNotePt2().withTimeout(6.0),
      fourNotePt3().withTimeout(6.0),
      new ArmPID(arm_, 32.85),
      new WaitCommand(0.2),
      new InstantCommand(() -> feeder_.setPercent(0.35)),
      new InstantCommand(() -> intake_.setPercent(0.5)),
      new WaitCommand(1.0),
      new ArmPID(arm_, 2),
      new WaitCommand(0.2),
      new InstantCommand(() -> shooter_.stopMotor()),
      new InstantCommand(() -> intake_.stopMotor()),
      new InstantCommand(() -> feeder_.stopMotor())
    );
  }

  public Command fourNotePt1() {
    return new ParallelCommandGroup(
      // Rev shooter, follow path to intake
      new InstantCommand(() -> shooter_.setPercent(0.75)),
      AutoBuilder.followPath(left_intake_path),
      new SequentialCommandGroup(
        new ArmPID(arm_, 24.9),
        new WaitCommand(0.5),
        new InstantCommand(() -> intake_.setPercent(0.55)),
        new InstantCommand(() -> feeder_.setPercent(0.3)),
        new WaitCommand(0.2),
        new ArmPID(arm_, 2),
        new InstantCommand(() -> intake_.setPercent(0.55)),
        new InstantCommand(() -> feeder_.stopMotor())
      )
    );
  }

  public Command fourNotePt2() {
    return new ParallelCommandGroup(
      // Rev shooter, follow path to shoot
      // new InstantCommand(() -> shooter_.setPercent(-0.75)),
      AutoBuilder.followPath(middle_intake_path),
      new SequentialCommandGroup(
        new ArmPID(arm_, 26.2),
        new WaitCommand(0.6),
        new InstantCommand(() -> intake_.setPercent(0.55)),
        new InstantCommand(() -> feeder_.setPercent(0.3)),
        new WaitCommand(0.5),
        new ArmPID(arm_, 2),
        new InstantCommand(() -> intake_.setPercent(0.55)),
        new InstantCommand(() -> feeder_.stopMotor())
      )
    );
  }

  public Command fourNotePt3() {
    return new ParallelCommandGroup(
      AutoBuilder.followPath(right_intake_path),
      new SequentialCommandGroup(
        new ArmPID(arm_, 26.4),
        new WaitCommand(0.5),
        new InstantCommand(() -> feeder_.setPercent(0.3)),
        new InstantCommand(() -> intake_.setPercent(0.55)),
        new WaitCommand(0.5),
        new ArmPID(arm_, 2).withTimeout(1.5),
        new InstantCommand(() -> feeder_.stopMotor()),
        new InstantCommand(() -> intake_.setPercent(0.55))
        
      )
    );
  }

  public Command fourNoteAuto() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        // Rev shooter, follow path to intake
        new InstantCommand(() -> shooter_.setPercent(-0.75)),
        AutoBuilder.followPath(left_intake_path),
        new SequentialCommandGroup(
          new ArmPID(arm_, 28),
          new WaitCommand(0.5),
          new InstantCommand(() -> intake_.setPercent(0.55)),
          new InstantCommand(() -> feeder_.setPercent(0.3)),
          new WaitCommand(0.2),
          new ArmPID(arm_, 2),
          new InstantCommand(() -> intake_.setPercent(0.55)),
          new InstantCommand(() -> feeder_.stopMotor())
        )
      ),
      new ParallelCommandGroup(
        // Rev shooter, follow path to shoot
        // new InstantCommand(() -> shooter_.setPercent(-0.75)),
        AutoBuilder.followPath(middle_intake_path),
        new SequentialCommandGroup(
          new ArmPID(arm_, 28.5),
          // new WaitCommand(0.5),
          new InstantCommand(() -> intake_.setPercent(-0.35)),
          new InstantCommand(() -> feeder_.setPercent(0.3)),
          new WaitCommand(0.5),
          new ArmPID(arm_, 2),
          new InstantCommand(() -> intake_.setPercent(-0.35)),
          new InstantCommand(() -> feeder_.stopMotor())
        )
      )
      // new ParallelCommandGroup(
      //   AutoBuilder.followPath(right_intake_path),
      //   new ArmPID(arm_, 32),
      //   new InstantCommand(() -> feeder_.setPercent(0.5)),
      //   new WaitCommand(0.5),
      //   new SequentialCommandGroup(
      //     new InstantCommand(() -> feeder_.stopMotor()),
      //     new InstantCommand(() -> intake_.setPercent(-0.75))
      //   ),
      //   new ArmPID(arm_, 2)
      // )
      // new ArmPID(arm_, 32),
      // new InstantCommand(() -> feeder_.setPercent(0.5)),
      // new WaitCommand(0.5),
      // new ParallelCommandGroup(
      //   new ArmPID(arm_, 2),
      //   new InstantCommand(() -> intake_.stopMotor()),
      //   new InstantCommand(() -> feeder_.stopMotor()),
      //   new InstantCommand(() -> shooter_.stopMotor())
      // )
    );
  }

  public Command shootMove() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        // Rev shooter, follow path to intake
        new InstantCommand(() -> shooter_.setPercent(0.75)),
        AutoBuilder.followPath(middle_middle_intake_path),
        new SequentialCommandGroup(
          new ArmPID(arm_, 30),
          new InstantCommand(() -> intake_.setPercent(0.5)),
          new InstantCommand(() -> feeder_.setPercent(0.3)),
          new WaitCommand(0.15),
          new ArmPID(arm_, 2),
          new InstantCommand(() -> intake_.setPercent(0.5)),
          new InstantCommand(() -> feeder_.stopMotor()),
          new InstantCommand(() -> shooter_.stopMotor()),
          new WaitCommand(2.0),
          new InstantCommand(() -> intake_.stopMotor())
        )
      )
    );
  }

  public Command shootTaxi() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> shooter_.setPercent(0.75)),
      new ArmPID(arm_, 16.5),
      new WaitCommand(0.4),
      new InstantCommand(() -> intake_.setPercent(0.5)),
      new InstantCommand(() -> feeder_.setPercent(0.3)),
      new WaitCommand(1.0),
      new ArmPID(arm_, 2),
      AutoBuilder.followPath(go_straight_path),
      new InstantCommand(() -> shooter_.stopMotor()),
      new InstantCommand(() -> intake_.stopMotor()),
      new InstantCommand(() -> feeder_.stopMotor())
    );
  }

  public Command threeNoteMiddleAuto() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ArmPID(arm_, 16.5),
        new InstantCommand(() -> shooter_.setPercent(0.75))
      ),
      new WaitCommand(0.2),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake_.setPercent(0.5)),
        new InstantCommand(() -> feeder_.setPercent(0.3))
      ),
      new WaitCommand(1.0),
      new ParallelCommandGroup(
        // new InstantCommand(() -> intake_.stopMotor()),
        new InstantCommand(() -> feeder_.stopMotor())
        // new InstantCommand(() -> shooter_.stopMotor())
      ),
      new ParallelCommandGroup(
        new ArmPID(arm_, 2),
        new InstantCommand(() -> intake_.setPercent(0.55)),
        AutoBuilder.followPath(middle_middle_intake_path)
      ),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        // new InstantCommand(() -> intake_.stopMotor()),
        new InstantCommand(() -> feeder_.stopMotor())
        // new InstantCommand(() -> shooter_.stopMotor())
      ),
      new ParallelCommandGroup(
        new ArmPID(arm_, 26.75),
        new InstantCommand(() -> shooter_.setPercent(0.75))
      ),
      new WaitCommand(0.8), // This one might be needed originally set to 1
      new ParallelCommandGroup(
        new InstantCommand(() -> intake_.setPercent(0.5)),
        new InstantCommand(() -> feeder_.setPercent(0.3))
      ),
      new WaitCommand(0.8),
      new ParallelCommandGroup(
        new ArmPID(arm_, 2),
        // new InstantCommand(() -> intake_.stopMotor()),
        new InstantCommand(() -> feeder_.stopMotor())
        // new InstantCommand(() -> shooter_.stopMotor())
      ),
      new WaitCommand(0.1),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake_.setPercent(0.55)),
        AutoBuilder.followPath(middle_left_intake_path)
      ),
      new WaitCommand(0.3),
      new ParallelCommandGroup(
        // new InstantCommand(() -> intake_.stopMotor()),
        new InstantCommand(() -> feeder_.stopMotor())
        // new InstantCommand(() -> shooter_.stopMotor())
      ),
      new ParallelCommandGroup(
        new InstantCommand(() -> shooter_.setPercent(0.75)),
        AutoBuilder.followPath(left_shoot_path),
        new ArmPID(arm_, 30.2)
      ),
      new WaitCommand(0.8),
      new ParallelCommandGroup(
        new InstantCommand(() -> intake_.setPercent(0.5)),
        new InstantCommand(() -> feeder_.setPercent(0.3))
      ),
      // End of Auto (moves arm down and turns off all motors)
      new WaitCommand(0.8),
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
      // AutoBuilder.followPath(left_one_intake_path)
    );
  }

  // Get starting pose of autonomous path
  public Pose2d getOneShotTaxiPose2d() {
    return go_straight_path.getStartingDifferentialPose();
  }

  public Pose2d getFourNotePose2d() {
    return left_intake_path.getStartingDifferentialPose();
  }

  public Pose2d getThreeNoteMiddlePose2d() {
    return middle_middle_intake_path.getStartingDifferentialPose();
  }


  public enum Routine {
    FOUR_NOTE_AUTO,
    THREE_NOTE_AUTO,
    ONE_NOTE_TAXI_AUTO
  }
  

  public static class Constants {
    public static final double kP = 5.0;
  }
}
