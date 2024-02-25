package org.ghrobotics.frc2024;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import org.ghrobotics.frc2024.commands.ArmToPosition;
import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Climber;
import org.ghrobotics.frc2024.subsystems.Intake;
import org.ghrobotics.frc2024.subsystems.Shooter;

public class Superstructure {
  // Subsystems
  private final Arm arm_;
  private final Climber climber_;
  private final Intake intake_;
  private final Shooter shooter_;

  //Store Position
  public String state = "STOW";

  // Constructor
  public Superstructure(Arm arm, Climber climber, Intake intake, Shooter shooter) {
    arm_ = arm;
    climber_ = climber;
    intake_ = intake;
    shooter_ = shooter;
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter Percent", shooter_.getPercent());
    SmartDashboard.putNumber("Intake Percent", intake_.getPercent());
  }

  // Position Setter
  public Command setPosition(Position pos) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> this.state = pos.posname),
      new ParallelCommandGroup(
        new ArmToPosition(arm_, pos.angle)
      ).withTimeout(2)
    );
  }

  // Intake Setter
  // Might change to functional later
  public Command setIntake(double percent) {
    return new StartEndCommand(
      () -> intake_.setPercent(percent),
      () -> intake_.setPercent(0),
      intake_
    );
  }

  // Shooter Setter
  public Command setShooter(double percent) {
    return new StartEndCommand(
      () -> shooter_.setPercent(percent),
      () -> shooter_.setPercent(0),
      shooter_
    );
  }

  // Jog Arm
  public Command jogArm(double percent) {
    return new StartEndCommand(
      () -> arm_.setPercent(percent),
      () -> arm_.setAngle(arm_.getAngle()),
      arm_
    );
  }

  // Jog Left Climber
  public Command jogLeftClimber(double percent) {
    return new StartEndCommand(
      () -> climber_.setLeftPercent(percent),
      () -> climber_.setLeftPercent(0.1/12),
      climber_
    );
  }

  // Jog Right Climber
  public Command jogRightClimber(double percent) {
    return new StartEndCommand(
      () -> climber_.setRightPercent(percent),
      () -> climber_.setRightPercent(0.1/12),
      climber_
    );
  }

  // GetPosition of Superstructure
  public String getState() {
    return state;
  }

  public enum Position {
    STOW(0, "STOW"),
    SUBWOOFER(0, "SUBWOOFER"),
    AMP(0, "AMP"),
    GROUND_INTAKE(0, "GROUND_INTAKE"),
    SOURCE_INTAKE(0, "SOURCE_INTAKE");
    
    final double angle;
    final String posname;

    Position(double angle_deg, String name) {
      this.angle = Math.toRadians(angle_deg);
      this.posname = name;
    }
  }
}