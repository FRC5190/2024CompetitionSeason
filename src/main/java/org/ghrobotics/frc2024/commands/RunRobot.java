// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024.commands;

import org.ghrobotics.frc2024.subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RunRobot extends Command {
  /** Creates a new RunRobot. */

  private final Drive drive_;

  Timer time_ = new Timer();
  public RunRobot(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive_ = drive;
    addRequirements(drive_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time_.reset();
    time_.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive_.setSpeeds(new ChassisSpeeds(1.5, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive_.setSpeeds(new ChassisSpeeds(0, 0, 0));
    time_.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time_.get() > 3.0;
  }
}
