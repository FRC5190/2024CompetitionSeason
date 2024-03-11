// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024.subsystems;

import org.ghrobotics.frc2024.LimelightHelpers;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
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
    // Read inputs
    io_.tv = table_.getEntry("tv").getDouble(0);
    io_.tx = table_.getEntry("tx").getDouble(0);
    io_.ty = table_.getEntry("ty").getDouble(0);
    io_.ta = table_.getEntry("ta").getDouble(0);
    io_.ts = table_.getEntry("ts").getDouble(0);
    io_.tl = table_.getEntry("tl").getDouble(0);
    io_.cl = table_.getEntry("cl").getDouble(0);
    io_.jsonString = table_.getEntry("json").getString("");
    io_.tid = table_.getEntry("tid").getDouble(0);
    io_.botpose = table_.getEntry("botpose").getDoubleArray(new double[6]);
    io_.botpose_wpiblue = table_.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    io_.botpose_wpired = table_.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    io_.camerapose_targetspace = table_.getEntry("camerapose_targetspace").getDoubleArray(
      new double[6]);
    io_.targetpose_cameraspace = table_.getEntry("targetpose_cameraspace").getDoubleArray(
      new double[6]);

    // Write outputs
    table_.getEntry("ledMode").setDouble(io_.led_mode);
    table_.getEntry("pipeline").setDouble(io_.pipeline);

    // Limelight
    io_.limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");

    
  }

  public boolean hasTarget() {
    return io_.tv != 0;
  }

  public Pose2d getEstimatedVisionRobotPose() {
    return io_.limelightMeasurement.pose;
  }

  public Pose2d getBotPose2d() {
    return LimelightHelpers.getBotPose2d_wpiBlue("limelight");
  }

  /**
   * @return Number of targets detected
   */
  public double getTv() {
    return io_.tv;
  }

  public double getTx() {
    return io_.tx;
  }

  public double getTy() {
    return io_.ty;
  }

  public double getArea() {
    return io_.ta;
  }

  public double getSkew() {
    return io_.ts;
  }

  public double getProcessingLatency() {
    return io_.tl;
  }

  public double getCaptureLatency() {
    return io_.cl;
  }

  public int getID() {
    return (int) io_.tid;
  }

  public int getFID() {
    return io_.fid;
  }

  public double[] getBotPose() {
    return io_.botpose;
  }

  public double[] getBlueBotPose() {
    return io_.botpose_wpiblue;
  }

  public double[] getCamPosTargetSpace() {
    return io_.camerapose_targetspace;
  }

  public double[] getTargetPosCamSpace() {
    return io_.targetpose_cameraspace;
  }

  public void setLED(LEDMode mode) {
    io_.led_mode = mode.ordinal();
  }

  public void setPipeline(int pipeline) {
    io_.pipeline = pipeline;
  }

  public enum LEDMode {
    PIPELINE, OFF, BLINK, ON
  }

  private static class PeriodicIO {
    // Inputs
    double tv;
    double tx;
    double ty;
    double ta;
    double ts;
    double tl;
    double cl;
    String jsonString;
    int fid;
    double tid;
    double[] botpose;
    double[] botpose_wpiblue;
    double[] botpose_wpired;
    double[] camerapose_targetspace;
    double[] targetpose_cameraspace;

    Pose2d botpose2d_wpired;

    LimelightHelpers.PoseEstimate limelightMeasurement;

    // Outputs
    double led_mode;
    double pipeline;
  }
}
