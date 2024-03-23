// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024;

import org.ghrobotics.frc2024.auto.AutoSelector;
import org.ghrobotics.frc2024.commands.ArmPID;
import org.ghrobotics.frc2024.commands.DriveTeleop;
import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Climber;
import org.ghrobotics.frc2024.subsystems.Drive;
import org.ghrobotics.frc2024.subsystems.Feeder;
import org.ghrobotics.frc2024.subsystems.Intake;
import org.ghrobotics.frc2024.subsystems.Limelight;
import org.ghrobotics.frc2024.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
  private final Climber climber_ = new Climber();
  private final Limelight limelight_ = new Limelight("limelight");

  private final Field2d field_ = new Field2d();

  private boolean isAuto;

  // Robot State
  private final RobotState robot_state_ = new RobotState(drive_, limelight_);

  private double brake_value_ = 0.05;
  
  // Controller
  private final CommandXboxController driver_controller_ = new CommandXboxController(0);
  private final CommandXboxController operator_controller_ = new CommandXboxController(1);

  // Superstructure
  private final Superstructure superstructure_ = new Superstructure(arm_, intake_, shooter_, feeder_, climber_);
  private final AutoSelector auto_selector_= new AutoSelector(drive_, robot_state_, superstructure_, arm_, intake_, shooter_, feeder_);

  // Telemetry
  private final Telemetry telemetry_ = new Telemetry(robot_state_, arm_, auto_selector_);

  @Override
  public void robotInit() {
    drive_.setDefaultCommand(new DriveTeleop(drive_, robot_state_, driver_controller_));

    SmartDashboard.putData("field", field_);
    // SmartDashboard.putData("Current Trajectory", (Sendable) auto_selector_.getPath());
    System.out.println("Robot Init");
    setupTeleopControls();

    // Just to test the blue subwoofer distance
    // robot_state_.reset(limelight_.getEstimatedVisionRobotPose());
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    superstructure_.periodic();
    // telemetry_.periodic();
    if(!Robot.isSimulation())
      robot_state_.update();

    SmartDashboard.putBoolean("Auto", isAuto);
    SmartDashboard.putNumber("Vision x", limelight_.getBotPose2d().getX());
    SmartDashboard.putNumber("Vision y", limelight_.getBotPose2d().getY());
    SmartDashboard.putNumber("Vision Degrees", limelight_.getBotPose2d().getRotation().getDegrees());

    SmartDashboard.putData(auto_selector_.getRoutineChooser());
    SmartDashboard.putData(auto_selector_.getPositionChooser());

    // if(!Robot.isSimulation())
    //   field_.setRobotPose(robot_state_.getPosition());
  }


  @Override
  public void autonomousInit() {
    isAuto = true;

    robot_state_.reset(auto_selector_.getSelectedPose2d());
    auto_selector_.getSelectedRoutine().schedule();

    drive_.setBrakeMode(true);
    arm_.setBrakeMode(true);
  }
  
  @Override
  public void autonomousPeriodic() {
    // if(Robot.isSimulation())
    //   telemetry_.simulationPeriodic();
  }
  

  @Override
  public void teleopInit() {
    isAuto = false;
    robot_state_.reset(limelight_.getEstimatedVisionRobotPose());
    drive_.setBrakeMode(true);
    arm_.setBrakeMode(true);
  }
  
  @Override
  public void teleopPeriodic() {}
  
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
    //  * RT:  Spin Shooter
    driver_controller_.rightTrigger().whileTrue(superstructure_.setShooterPercent(0.85));

    driver_controller_.rightBumper().whileTrue(superstructure_.setShooterPercent(-0.6));

    // driver_controller_.pov(180).whileTrue(superstructure_.setShooter(0.55));

    // driver_controller_.pov(270).whileTrue(superstructure_.setShooter(85));

    driver_controller_.leftTrigger().whileTrue(superstructure_.setIntake(0.55));

    driver_controller_.leftBumper().whileTrue(superstructure_.setIntake(-0.25));

    driver_controller_.pov(0).whileTrue(superstructure_.setArmPercent(0.056));

    driver_controller_.a().whileTrue(superstructure_.setFeeder(0.85));

    driver_controller_.b().whileTrue(superstructure_.setFeeder(-0.75));

    driver_controller_.y().onTrue(new InstantCommand(() -> drive_.resetGyro()));

    // driver_controller_.pov(90).whileTrue(superstructure_.setShooter(90));
    
    driver_controller_.pov(90).whileTrue(superstructure_.setArmPercent(0.15));

    driver_controller_.pov(270).whileTrue(superstructure_.setArmPercent(-0.15));

    
    // Operator Control

    operator_controller_.leftTrigger().whileTrue(superstructure_.setArmPercent(brake_value_));

    operator_controller_.b().onTrue(superstructure_.setPosition(Superstructure.Position.SUBWOOFER));

    operator_controller_.a().onTrue(superstructure_.setPosition(Superstructure.Position.GROUND_INTAKE));

    operator_controller_.y().onTrue(superstructure_.setPosition(Superstructure.Position.STOW));

    operator_controller_.x().onTrue(new ArmPID(arm_, 35));

    operator_controller_.leftBumper().whileTrue(superstructure_.setLeftClimber(0.5));

     operator_controller_.rightBumper().whileTrue(superstructure_.setRightClimber(0.8));

     operator_controller_.leftTrigger().whileTrue(superstructure_.setLeftClimber(-0.5));

     operator_controller_.rightTrigger().whileTrue(superstructure_.setRightClimber(-0.8));

    operator_controller_.pov(0).whileTrue(superstructure_.setArmPercent(0.25));
    
    operator_controller_.pov(180).whileTrue(superstructure_.setArmPercent(-0.25));
  }

  public class Constatnts {
    // Red Subwoofer Pose 2d
    public static Pose2d redSubwooferPose = new Pose2d(16.5, 5.57, new Rotation2d(0));

    // Blue Subwoofer Pose 2d
    public static Pose2d blueSubwooferPose = new Pose2d(0.05, 5.57, new Rotation2d(0));
  }
}
