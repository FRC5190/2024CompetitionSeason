package org.ghrobotics.frc2024;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.ghrobotics.frc2024.commands.ArmPID;
import org.ghrobotics.frc2024.commands.ArmToPosition;
import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Feeder;
// import org.ghrobotics.frc2024.subsystems.Climber;
import org.ghrobotics.frc2024.subsystems.Intake;
import org.ghrobotics.frc2024.subsystems.Shooter;

public class Superstructure {
  // Subsystems
  private final Arm arm_;
  // private final Climber climber_;
  private final Intake intake_;
  private final Shooter shooter_;
  private final Feeder feeder_;

  //Store Position
  public String state = "STOW";

  // Constructor
  public Superstructure(Arm arm, Intake intake, Shooter shooter, Feeder feeder) {
    arm_ = arm;
    // climber_ = climber;
    intake_ = intake;
    shooter_ = shooter;
    feeder_ = feeder;
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter Percent", shooter_.getPercent());
    // SmartDashboard.putNumber("Intake Percent", intake_.getPercent());
    SmartDashboard.putNumber("Arm Angle", Math.toDegrees(arm_.getAngle()));
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

  public Command setAnglePosition(double angle) {
    return new SequentialCommandGroup(
      new ArmPID(arm_, angle)
    ).withTimeout(0.5);
  }

  // Intake Setter
  // Might change to functional later
  public Command setIntake(double percent) {
    return new StartEndCommand(
      () -> intake_.setPercent(percent),
      () -> intake_.stopMotor(),
      intake_
    );
  }

  // Shooter Setter
  public Command setShooter(double percent) {
    return new StartEndCommand(
      () -> shooter_.setPercent(percent),
      () -> shooter_.stopMotor(),
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

  public Command shoot() {
    return Commands.parallel(setIntake(-0.6), setShooter(-0.75));
  }

  /**
   * Feeder Setter
   * @param percent
   */
  public Command setFeeder(double percent) {
    return new StartEndCommand(
      () -> feeder_.setPercent(percent),
      () -> feeder_.setPercent(0),
      feeder_
    );
  }

  public Command shootDelay() {
    return new StartEndCommand(
      () -> {
        setShooter(0.75);
        new WaitCommand(0.5);
        Commands.parallel(setIntake(-0.6), setFeeder(-0.75));
      },
      () -> {
        new InstantCommand(() -> {
          shooter_.stopMotor();
          intake_.stopMotor();
          feeder_.stopMotor();
        });
      },
      feeder_  
    );
  }

  public Command setArmPercent(double percent) {
    return new StartEndCommand(
      () -> arm_.setPercent(percent),
      () -> arm_.setPercent(0),
      arm_
    );
  }


  public Command setArmPID(double angle) {
    return new StartEndCommand(
      () -> arm_.setAnglePID(angle),
      () -> arm_.setPercent(0),
      arm_
    );
  }

  // // Jog Left Climber
  // public Command jogLeftClimber(double percent) {
  //   return new StartEndCommand(
  //     () -> climber_.setLeftPercent(percent),
  //     () -> climber_.setLeftPercent(0.1/12),
  //     climber_
  //   );
  // }

  // // Jog Right Climber
  // public Command jogRightClimber(double percent) {
  //   return new StartEndCommand(
  //     () -> climber_.setRightPercent(percent),
  //     () -> climber_.setRightPercent(0.1/12),
  //     climber_
  //   );
  // }

  // GetPosition of Superstructure
  public String getState() {
    return state;
  }

  public enum Position {
    STOW(-35, "STOW"),
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
