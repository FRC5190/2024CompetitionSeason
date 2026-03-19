package org.ghrobotics.frc2024.simulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class SimulateDrive{
  private final CommandXboxController controller_ = new CommandXboxController(0);;

  private final Field2d field_ = new Field2d();


  private final IO io_ = new IO();


  // Constructor
  public SimulateDrive() {
      // Assign member variables
      SmartDashboard.putData("Field", field_);

  }

  public void initialize() {
      field_.setRobotPose(3.5, 3.5, Rotation2d.fromDegrees(0.0));
  }


  public void periodic() {
      // Log everything
      SmartDashboard.putNumber("Controller X", controller_.getLeftX());
      SmartDashboard.putNumber("Controller Y", controller_.getLeftY());
      SmartDashboard.putNumber("Controller Angle", controller_.getRightX());
      SmartDashboard.putNumber("Controller R2 Axis", controller_.getRightTriggerAxis());
      SmartDashboard.putNumber("X Position", io_.x_position);
      SmartDashboard.putNumber("Y Position", io_.y_position);
      SmartDashboard.putNumber("Angle", io_.angle);

      // SmartDashboard.putNumber("PID Turn", turn_controller_.calculate(field_.getRobotPose().getRotation().getDegrees(), new Rotation2d(0.0).getDegrees()));


      io_.x_position = field_.getRobotPose().getTranslation().getX();
      io_.y_position = field_.getRobotPose().getTranslation().getY();
      io_.angle = field_.getRobotPose().getRotation().getDegrees();


      io_.speed_x = deadband(controller_.getLeftX());
      io_.speed_y = deadband(-controller_.getLeftY());
      io_.speed_angle = deadband(controller_.getRightX() * -180);

      io_.x_position += io_.speed_x * Constants.kMaxSpeed;
      io_.y_position += io_.speed_y * Constants.kMaxSpeed;
      io_.angle += io_.speed_angle * Constants.kMaxSpeed;

      field_.setRobotPose(io_.x_position, io_.y_position, Rotation2d.fromDegrees(io_.angle));
  }

  public double getXPosition() {
    return io_.x_position;
  }
  public double getYPosition() {
    return io_.y_position;
  }
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(io_.angle);
  }


  // Controller Deadband
  public static double deadband(double input) {
      if (Math.abs(input) < 0.075) {
          return 0.0;
      }
      return input;
  }

  private static class IO {
      public double x_position;
      public double y_position;
      public double angle;


      public double speed_x;
      public double speed_y;
      public double speed_angle;
  }

  private static class Constants {
    public static final double kMaxSpeed = 0.01;
    public static final double kP = 0.1;
  }
  
}
