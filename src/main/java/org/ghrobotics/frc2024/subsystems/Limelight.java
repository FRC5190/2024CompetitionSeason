// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  // Get network table
  NetworkTable table_ = NetworkTableInstance.getDefault().getTable("limelight");

  // IO
  private final PeriodicIO io_ = new PeriodicIO();


  /** Creates a new Limelight. */
  public Limelight(String name) {
    table_ = NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io_.tx = table_.getEntry("tx").getDouble(0.0);
    io_.ty = table_.getEntry("ty").getDouble(0.0);

    // Change this 'getBotPoseEstimate_wpiBlue()'
    io_.botpose_wpiblue = table_.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
  }

  private static class PeriodicIO {
    // Inputs
    double tx;
    double ty;
    double [] botpose_wpiblue;
  }
}
