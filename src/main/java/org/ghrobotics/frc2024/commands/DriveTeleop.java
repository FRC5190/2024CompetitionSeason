// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024.commands;

import org.ghrobotics.frc2024.RobotState;
import org.ghrobotics.frc2024.subsystems.Drive;
import org.ghrobotics.frc2024.RobotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveTeleop extends Command {
  // Subsystems
  private final Drive drive_;
  private final RobotState robot_state_;

  // Xbox Controller
  private final CommandXboxController controller_;

  // Constructor
  public DriveTeleop(Drive drive, RobotState robot_state, CommandXboxController controller) {
    // Assign member variables
    drive_ = drive;
    robot_state_ = robot_state;
    controller_ = controller;

    // Add subsystem requirements
    addRequirements(drive_);
  }

  @Override
  public void execute() {
    // Inputs input

    double xSpeed = -controller_.getLeftY() * Constants.kTranslationMultiplier;
    double ySpeed = -controller_.getLeftX() * Constants.kTranslationMultiplier;
    double rSpeed = -controller_.getRightX() * Constants.kRotationMultiplier;
    boolean robot_oriented = controller_.rightTrigger().getAsBoolean();
    boolean hold_position_mode = controller_.x().getAsBoolean();

    if(Math.abs((xSpeed)) < 0.02) {
      xSpeed = 0;
    }

    if (Math.abs((ySpeed)) < 0.02) {
      ySpeed = 0;
    }

    if (Math.abs((rSpeed)) < 0.02) {
      rSpeed = 0;
    }

    // Create chassis speeds
    ChassisSpeeds speeds = robot_oriented ?
      new ChassisSpeeds(xSpeed, ySpeed, rSpeed) :
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed,
        drive_.getAngle());

    // Set speeds
    if (hold_position_mode) {
      drive_.HoldPosition();
    }
    else{
      drive_.setSpeeds(speeds, Drive.OutputType.OPEN_LOOP);
    }

    if (controller_.leftBumper().getAsBoolean())
      robot_state_.reset(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
  }

  // Constants
  private static class Constants {
    // Joystick Multiplier (percent -> speed)
    public static final double kTranslationMultiplier = 2.5;
    public static final double kRotationMultiplier = Math.PI;
  }
}
