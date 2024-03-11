// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024.commands;

import java.sql.Driver;

import org.ghrobotics.frc2024.RobotState;
import org.ghrobotics.frc2024.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToSpeaker extends Command {
  private final Drive drive_;
  private final RobotState robot_state_;

  private final PIDController pid_ = new PIDController(Constants.kP, 0, 0);

  public double targetAngle;
  



  /** Creates a new RotateToSpeaker. */
  public RotateToSpeaker(Drive drive, RobotState robot_state) {
    drive_ = drive;
    robot_state_ = robot_state;
    
    addRequirements(drive_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // pid_.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the current robot pose
    
    SmartDashboard.putNumber("1Difference in Angle", Math.abs(targetAngle - robot_state_.getPosition().getRotation().getDegrees()));

    drive_.setsOpenSpeeds(new ChassisSpeeds(0, 0, calculateAngleToSpeaker()));
  }

  public double calculateAngleToSpeaker() {
    Pose2d targPose2d;
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      targPose2d = Constants.redSubwooferPose;
    } else {
      targPose2d = Constants.blueSubwooferPose;
    }
    targetAngle = 180.0;

    targetAngle = Math.toDegrees(
      Math.atan2(
        targPose2d.getTranslation().getY() - robot_state_.getPosition().getTranslation().getY(), 
        targPose2d.getTranslation().getX() - robot_state_.getPosition().getTranslation().getX()));
    

    return MathUtil.clamp(pid_.calculate(robot_state_.getPosition().getRotation().getDegrees(), targetAngle), -1.0, 1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive_.setsOpenSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(targetAngle - robot_state_.getPosition().getRotation().getDegrees()) < 2.0);
  }

  private static class Constants {
    // PID Constants
    public static double kP = 0.01;

    // Red Subwoofer Pose 2d
    public static Pose2d redSubwooferPose = new Pose2d(16.5, 5.57, new Rotation2d(0));

    // Blue Subwoofer Pose 2d
    public static Pose2d blueSubwooferPose = new Pose2d(0.05, 5.57, new Rotation2d(0));
  }
}
