package org.ghrobotics.frc2024;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import java.util.function.DoubleSupplier;

import org.ghrobotics.frc2024.commands.ArmPID;
import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Feeder;
import org.ghrobotics.frc2024.subsystems.Intake;
import org.ghrobotics.frc2024.subsystems.Shooter;

public class Superstructure {
  // Subsystems
  private final Arm arm_;
  private final Intake intake_;
  private final Shooter shooter_;
  private final Feeder feeder_;
  private final RobotState robot_state_;
  // private final ShootingPosition shootingPosition_ = new ShootingPosition();
  

  //Store Position
  public String state = "STOW";

  // Shooting Angle
  public double armShootingAngle;
  public double shootingDistance;

  // Constructor
  public Superstructure(Arm arm, Intake intake, Shooter shooter, Feeder feeder, RobotState robot_state) {
    arm_ = arm;
    intake_ = intake;
    shooter_ = shooter;
    feeder_ = feeder;
    robot_state_ = robot_state;
  }

  public void periodic() {
    // shootingDistance = shootingPosition_.distanceToSpeaker(robot_state_.getPosition(), ShootingPosition.Constants.kBlueSubwooferPose);
    SmartDashboard.putNumber("Distance from speaker", shootingDistance);
    // armShootingAngle = shootingPosition_.regressionFormula(shootingDistance);

    SmartDashboard.putNumber("Arm Angle", Math.toDegrees(arm_.getAngle()));

    SmartDashboard.putNumber("Shooting Angle", armShootingAngle);

    SmartDashboard.putString("Superstructure State", state);

    // Checks output current to see if note has intaked or not (current > 35 means intaked)
    if (intake_.getRightOutputCurrent() > 45) {
      LimelightHelpers.setLEDMode_ForceOn("limelight");
    }
  }

  // Position Setter
  public Command setPosition(Position pos) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> this.state = pos.posname),
      new ParallelCommandGroup(
        new ArmPID(arm_, pos.angle)
      ).withTimeout(3.5)
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
  public Command setShooterPercent(double percent) {
    return new StartEndCommand(
      () -> shooter_.setPercent(percent),
      () -> shooter_.stopMotor(),
      shooter_
    );
  }

  /**
   * Set Shooter Velocity
   * @param velocity in rotations per minute
   */
  public Command setShooter(double velocity) {
    return new StartEndCommand(
      () -> shooter_.setVelocity(velocity),
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
   * Keep arm at shooting angle
   * @param angle_deg angle in degrees
   */
  public Command autoArm(double angle_deg) {
    return new StartEndCommand(
      () -> arm_.setAnglePID(Math.toRadians(angle_deg)),
      () -> new ArmPID(arm_, 2),
      arm_
    );
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

  public Command setArmPercent(double percent) {
    return new StartEndCommand(
      () -> arm_.setPercent(percent),
      () -> {
        arm_.setPercent(0);
        arm_.setBrake(Math.toDegrees(arm_.getAngle()));
      },
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

  // GetPosition of Superstructure
  public String getState() {
    return state;
  }

  public enum Position {
    STOW(50, "STOW"),
    SUBWOOFER(16.5, "SUBWOOFER"),
    AMP(60, "AMP"),
    GROUND_INTAKE(2, "GROUND_INTAKE"),
    SOURCE_INTAKE(45, "SOURCE_INTAKE");
    
    final double angle;
    final String posname;

    Position(double angle_deg, String name) {
      this.angle = angle_deg;
      this.posname = name;
    }
  }
}
