package org.ghrobotics.frc2024.auto;

import org.ghrobotics.frc2024.RobotState;
import org.ghrobotics.frc2024.Superstructure;
import org.ghrobotics.frc2024.commands.ArmPID;
import org.ghrobotics.frc2024.commands.RunRobot;
import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Drive;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoSelector {
  // private final SendableChooser<Command> auto_chooser_;
  
  private final Drive drive_;
  private final RobotState robot_state_;
  private final Superstructure superstructure_;
  private final Arm arm_;


  Timer time_ = new Timer();

  public AutoSelector(Drive drive, RobotState robot_state, Superstructure superstructure, Arm arm) {
    
    drive_ = drive;
    robot_state_ = robot_state;
    superstructure_ = superstructure;
    arm_ = arm;
    
    

    // AutoBuilder.configureHolonomic(
    //   robot_state_::getPosition,
    //   robot_state_::reset,
    //   drive_::getSpeeds,
    //   drive_::setSpeeds,
    //   new HolonomicPathFollowerConfig(
    //     new PIDConstants(Constants.kP, 0.0, 0.0),
    //     new PIDConstants(Constants.kP, 0.0, 0.0),
    //     3.0,
    //     0.375,
    //     new ReplanningConfig()
    //   ),
    //   () -> {
    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //       return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    //   },
    //   drive_
    // );

    // auto_chooser_ = AutoBuilder.buildAutoChooser();
    // auto_chooser_.setDefaultOption("One Note Middle Taxi", new OneNoteMiddleTaxi(drive_, superstructure_));
    // auto_chooser_.addOption("One Note Top Taxi", new OneNoteTopTaxi(drive_, superstructure_));
    // auto_chooser_.addOption("One Note Bottom Taxi", new OneNoteBottomTaxi(drive_, superstructure_));
    // auto_chooser_.addOption("Two Note Top", new TwoNoteTop(drive_, superstructure_));
    // auto_chooser_.addOption("Two Note Middle", new TwoNoteMiddle(drive_, superstructure_));
    // auto_chooser_.addOption("Two Note Bottom", new TwoNoteBottom(drive_, superstructure_));

    // SmartDashboard.putData("Auto Chooser", auto_chooser_);
  }

  // public Command getAutonomousCommand() {
  //   return auto_chooser_.getSelected();
  // }

  public Command basicAutoCommand() {
    return moveCommand();
  }

  public Command moveCommand() {
    return new SequentialCommandGroup(
      new ArmPID(arm_, 2).withTimeout(1.5),
      new RunRobot(drive_)
    );
  }

  // this is here just so it stopped giving me errors we need to fix this
  // PathPlannerPath pickup_top = PathPlannerPath.fromPathFile("Pickup Top");
  // PathPlannerPath pickup_middle = PathPlannerPath.fromPathFile("Pickup Middle");
  // PathPlannerPath pickup_bottom = PathPlannerPath.fromPathFile("Pickup Bottom");

  // public Pose2d getStartPosition() {
  //   if (auto_chooser_.getSelected() instanceof TwoNoteBottom || auto_chooser_.getSelected() instanceof OneNoteBottomTaxi) {
  //     return pickup_bottom.getStartingDifferentialPose();
  //   }
  //   else if (auto_chooser_.getSelected() instanceof TwoNoteMiddle || auto_chooser_.getSelected() instanceof OneNoteMiddleTaxi) {
  //     return pickup_middle.getStartingDifferentialPose();
  //   }
  //   else if (auto_chooser_.getSelected() instanceof TwoNoteTop || auto_chooser_.getSelected() instanceof OneNoteTopTaxi) {
  //     return pickup_top.getStartingDifferentialPose();
  //   }
  //   return pickup_top.getStartingDifferentialPose();
  // }

  public static class Constants {
    public static double kP = 4.5;
  }
}
