package org.ghrobotics.frc2024;

import org.ghrobotics.frc2024.subsystems.Drive;
import org.ghrobotics.frc2024.subsystems.Limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
  // Swerve Drive
  private final Drive drive_;

  // Position Estimator
  private final SwerveDrivePoseEstimator pose_estimator_;

  private final Limelight limelight_;

  // Constructor
  public RobotState(Drive drive, Limelight limelight) {
    // Assign member variables
    drive_ = drive;
    limelight_ = limelight;

    // Initialize pose estimator
    pose_estimator_ = new SwerveDrivePoseEstimator(
      drive_.getKinematics(), drive.getAngle(), drive.getSwerveModulePositions(),
      new Pose2d());
  }

  // Update
  public void update() {
    // Update pose estimator with new drive measurements
    pose_estimator_.update(drive_.getAngle(), drive_.getSwerveModulePositions());

    // if (limelight_.getTv() >= 1) {
    //   pose_estimator_.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    //   pose_estimator_.addVisionMeasurement(
    //     limelight_.getEstimatedVisionRobotPose(), 
    //     limelight_.getProcessingLatency());
    // }

    SmartDashboard.putNumber("Robot Pose X", getPosition().getTranslation().getX());
    SmartDashboard.putNumber("Robot Pose Y", getPosition().getTranslation().getY());
    SmartDashboard.putNumber("Robot Pose Degrees", getPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("Actual Robot Heading", drive_.getAngle().getDegrees());
    SmartDashboard.putNumber("Estimated Robot Heading", getDegree());

    // SmartDashboard.putNumber("Vision X", limelight_.getEstimatedVisionRobotPose().getX());
    // SmartDashboard.putNumber("Vision Y", limelight_.getEstimatedVisionRobotPose().getY());
  }

  // Get Position
  public Pose2d getPosition() {
    return pose_estimator_.getEstimatedPosition();
  }

  // Get Degree
  public double getDegree() {
    return pose_estimator_.getEstimatedPosition().getRotation().getDegrees();
  }

  // Get Rotation2d
  public Rotation2d getRotation2d() {
    return pose_estimator_.getEstimatedPosition().getRotation();
  }

  // Reset Position
  // Add this after testing
  public void reset(Pose2d pose) {
    pose_estimator_.resetPosition(drive_.getAngle(), drive_.getSwerveModulePositions(), pose);
  }
}
