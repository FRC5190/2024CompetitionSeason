package org.ghrobotics.frc2024;

import java.util.ArrayList;
import java.util.List;

import org.ghrobotics.frc2024.auto.AutoSelector;
import org.ghrobotics.frc2024.subsystems.Arm;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {

  // Periodic
  private final List<Runnable> periodic_registry_ = new ArrayList<>();

  private final Field2d field_;

  private final Mechanism2d superstructure_;

  private final AutoSelector auto_selector_;


  // Timer for trajectory
  Timer timer_ = new Timer();


  public Telemetry(RobotState robot_state, Arm arm, AutoSelector auto_selector) {
    auto_selector_ = auto_selector;
    ShuffleboardTab tab_ = Shuffleboard.getTab("2024");

    tab_.add(auto_selector.getRoutineChooser());
    tab_.add(auto_selector.getPositionChooser());
    
    field_ = new Field2d();
        tab_.add("Field", field_);
        if (Robot.isReal())
            periodic_registry_.add(() -> field_.setRobotPose(robot_state.getPosition()));
        if(!Robot.isReal())
            field_.setRobotPose(new Pose2d(3, 3, Rotation2d.fromDegrees(0)));

    superstructure_ = new Mechanism2d(1.2, 1.2);

    tab_.add("Superstructure", superstructure_);

    superstructure_.getRoot("Structure Root", 0.33, 0.1).append(
      new MechanismLigament2d("Structure", 0.5, 0));
    MechanismLigament2d arm_out = superstructure_.getRoot("Carriage Root", 0.80, 0.15).append(
      new MechanismLigament2d("Arm", 0.15, 0, 3, new Color8Bit(Color.kOrangeRed)));

    periodic_registry_.add(() -> arm_out.setAngle(Math.toDegrees(arm.getAngle())));
  }

  public void periodic() {
    for (Runnable fn : periodic_registry_)
      fn.run();
    // System.out.println(periodic_registry_);

    // SmartDashboard.putData((Sendable) periodic_registry_);

  }

  public void simulationPeriodic() {
    // // PathPlannerTrajectory simulate_trajectory = auto_selector_.getPath().
    //   getTrajectory(new ChassisSpeeds(), auto_selector_.getStartingPose().getRotation());
    // timer_.reset();
    // timer_.start();

    // while (timer_.get() <= simulate_trajectory.getTotalTimeSeconds()) {
    //   State state = simulate_trajectory.sample(timer_.get());

    //   field_.setRobotPose(state.getTargetHolonomicPose());
    // }
  }

}
