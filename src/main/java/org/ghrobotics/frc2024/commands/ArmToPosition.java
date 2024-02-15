package org.ghrobotics.frc2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.ghrobotics.frc2024.subsystems.Arm;

public class ArmToPosition extends Command {
  // Subsystems
  private final Arm arm_;

  // Position
  private final double position_;

  // Constructor
  public ArmToPosition(Arm arm, double position) {
    // Assign member variables
    arm_ = arm;
    position_ = position;

    // Add subsystem requirements
    addRequirements(arm_);
  }

  @Override
  public void initialize() {
    arm_.setAngle(position_);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(arm_.getAngle() - position_) < Constants.kTolerance;
  }

  // Constants
  private static final class Constants {
    public static final double kTolerance = 0; // NEED TO UPDATE
  }
}