// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ghrobotics.frc2024.auto.AutoSelector;
import org.ghrobotics.frc2024.commands.ArmPID;
import org.ghrobotics.frc2024.commands.DriveTeleop;
import org.ghrobotics.frc2024.commands.RotateToSpeaker;
import org.ghrobotics.frc2024.subsystems.Arm;
// import org.ghrobotics.frc2024.subsystems.Climber;
import org.ghrobotics.frc2024.subsystems.Drive;
import org.ghrobotics.frc2024.subsystems.Feeder;
import org.ghrobotics.frc2024.subsystems.Intake;
import org.ghrobotics.frc2024.subsystems.Limelight;
import org.ghrobotics.frc2024.subsystems.Shooter;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
  // Subsystems
  private final Drive drive_ = new Drive();
  private final Arm arm_ = new Arm();
  private final Intake intake_ = new Intake();
  private final Shooter shooter_ = new Shooter();
  private final Feeder feeder_ = new Feeder();
  private final Limelight limelight_ = new Limelight("limelight");

  private final Field2d field_ = new Field2d();

  private boolean isAuto = false;

  // Robot State
  private final RobotState robot_state_ = new RobotState(drive_, limelight_);

  // Telemetry
  private final Telemetry telemetry_ = new Telemetry(robot_state_, arm_);

  private double brake_value_ = 0.05;
  
  // Controller
  private final CommandXboxController driver_controller_ = new CommandXboxController(0);
  private final CommandXboxController operator_controller_ = new CommandXboxController(1);

  // Superstructure
  private final Superstructure superstructure_ = new Superstructure(arm_, intake_, shooter_, feeder_, limelight_);
  private final AutoSelector auto_selector_= new AutoSelector(drive_, robot_state_, superstructure_, arm_, intake_, shooter_, feeder_);

  @Override
  public void robotInit() {
    drive_.setDefaultCommand(new DriveTeleop(drive_, robot_state_, driver_controller_));

    SmartDashboard.putData("field", field_);
    setupTeleopControls();
    // drive_.setBrakeMode(true);
    robot_state_.reset(auto_selector_.getStartingPose());
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    superstructure_.periodic();
    telemetry_.periodic();
    robot_state_.update();
    SmartDashboard.putBoolean("Auto", isAuto);

    SmartDashboard.putNumber("Robot Angle", drive_.getAngle().getDegrees());

    SmartDashboard.putNumber("estimated angle", robot_state_.getDegree());

    SmartDashboard.putNumber("Vision x", limelight_.getBotPose2d().getX());
    SmartDashboard.putNumber("Vision y", limelight_.getBotPose2d().getY());
    SmartDashboard.putNumber("Vision Degrees", limelight_.getBotPose2d().getRotation().getDegrees());

    field_.setRobotPose(limelight_.getBotPose2d());
  }


  @Override
  public void autonomousInit() {
    isAuto = true;

    robot_state_.reset(new Pose2d(
      auto_selector_.getStartingPose().getX(), 
      auto_selector_.getStartingPose().getY(), 
      Rotation2d.fromDegrees(0)));
    auto_selector_.followPath().schedule();

    drive_.setBrakeMode(true);
    arm_.setBrakeMode(true);
  }
  
  @Override
  public void autonomousPeriodic() {}
  

  @Override
  public void teleopInit() {
    isAuto = false;
    robot_state_.reset(limelight_.getBotPose2d());
    drive_.setBrakeMode(true);
    arm_.setBrakeMode(true);
  }
  
  @Override
  public void teleopPeriodic() {
    if (Math.toDegrees(arm_.getAngle()) < 15 && Math.toDegrees(arm_.getAngle()) > 0) {
      brake_value_ = 0.05;
    }

    if (Math.toDegrees(arm_.getAngle()) > 15 && Math.toDegrees(arm_.getAngle()) < 40) {
      brake_value_ = 0.05;
    }

    if (Math.toDegrees(arm_.getAngle()) > 40) {
      brake_value_ = 0.02;
    }

  }
  
  @Override
  public void disabledInit() {
    drive_.setBrakeMode(false);
    arm_.setBrakeMode(false);
  }
  
  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void testInit() {}
  
  @Override
  public void testPeriodic() {}
  
  @Override
  public void simulationInit() {}
  
  @Override
  public void simulationPeriodic() {}

  private void setupTeleopControls() {

    // Driver Control
    driver_controller_.rightTrigger().whileTrue(superstructure_.setShooter(-0.75));

    driver_controller_.rightBumper().whileTrue(superstructure_.setShooter(-0.5));

    driver_controller_.leftTrigger().whileTrue(superstructure_.setIntake(-0.25));

    driver_controller_.leftBumper().whileTrue(superstructure_.setIntake(0.15));

    driver_controller_.pov(0).whileTrue(superstructure_.setArmPercent(0.056));

    // driver_controller_.pov(180).whileTrue(superstructure_.setShooter(0.3));

    driver_controller_.b().whileTrue(superstructure_.shoot());

    driver_controller_.a().whileTrue(superstructure_.setFeeder(0.5));

    driver_controller_.pov(90).onTrue(new RotateToSpeaker(drive_, robot_state_, limelight_));

    // driver_controller_.a().whileTrue(superstructure_.setFeeder(0.5));

    
    // Operator Control

    operator_controller_.leftTrigger().whileTrue(superstructure_.setArmPercent(brake_value_));

    operator_controller_.b().onTrue(new ArmPID(arm_, 20));

    operator_controller_.a().onTrue(new ArmPID(arm_, 2));

    operator_controller_.y().onTrue(new ArmPID(arm_, 36));

    operator_controller_.x().onTrue(new ArmPID(arm_, 60));

    operator_controller_.pov(0).whileTrue(superstructure_.setArmPercent(0.1));
    
    operator_controller_.pov(180).whileTrue(superstructure_.setArmPercent(-0.1));
  }
}
