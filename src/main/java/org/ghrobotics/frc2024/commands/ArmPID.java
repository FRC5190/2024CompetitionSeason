package org.ghrobotics.frc2024.commands;

import org.ghrobotics.frc2024.subsystems.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmPID extends Command {
  // Subsystems
  private final Arm arm_;

  // Position
  private final double position_;

  private final Timer timer_ = new Timer();

  private double something;

  /**
   * Moves Arm to desired angle
   * @param arm
   * @param position Desired angle in degrees
   */
  public ArmPID(Arm arm, double position) {
    // Assign member variables
    arm_ = arm;
    position_ = position;

    // Add subsystem requirements
    addRequirements(arm_);
  }

  @Override
  public void initialize() {
    timer_.reset();
    timer_.start();
  }

  @Override
  public void execute() {
    arm_.setAnglePID(position_);
    something = Math.toDegrees(arm_.getAngle());
    SmartDashboard.putNumber("Somethig", something);
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Finished", Math.abs(Math.toDegrees(arm_.getAngle()) - position_) < Constants.kTolerance);
    SmartDashboard.putNumber("Finished get angle", Math.abs(arm_.getAngle()));
    return Math.abs(Math.toDegrees(arm_.getAngle()) - position_) < Constants.kTolerance;
  }

  @Override
  public void end(boolean interrupted) {
    arm_.setBrake(Math.toDegrees(arm_.getAngle()));
    timer_.reset();
  }

  // Constants
  private static final class Constants {
    public static final double kTolerance = 0.5; // NEED TO UPDATE
  }
}
