// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.ghrobotics.frc2024.Superstructure.Position;
import org.ghrobotics.frc2024.commands.DriveTeleop;
import org.ghrobotics.frc2024.subsystems.Arm;
// import org.ghrobotics.frc2024.subsystems.Climber;
import org.ghrobotics.frc2024.subsystems.Drive;
import org.ghrobotics.frc2024.subsystems.Intake;
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
  // private final Climber climber_ = new Climber();
  private final Intake intake_ = new Intake();
  private final Shooter shooter_ = new Shooter();
  
  // Robot State
  private final RobotState robot_state_ = new RobotState(drive_);

  // Telemetry
  private final Telemetry telemetry_ = new Telemetry(robot_state_, arm_);
  
  // Xbox Controller
  private final CommandXboxController driver_controller_ = new CommandXboxController(1);

  // Playstation controller just for testing
  private final CommandPS4Controller ps4_controller_ = new CommandPS4Controller(0);
  // Superstructure
  private final Superstructure superstructure_ = new Superstructure(arm_, intake_, shooter_);
  

  public Command test() {
    return new SequentialCommandGroup(
      superstructure_.setPosition(Position.STOW),
      new WaitCommand(3.0)
    );
  }
  @Override
  public void robotInit() {
    drive_.setDefaultCommand(new DriveTeleop(drive_, robot_state_, driver_controller_));

    setupTeleopControls();
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    superstructure_.periodic();
    telemetry_.periodic();

    SmartDashboard.putNumber("Leader Encoder angle deg", Math.toDegrees(arm_.getLeaderAngle()));
    SmartDashboard.putNumber("Follower Encoder angle deg", Math.toDegrees(arm_.getFollowerAngle()));
    SmartDashboard.putNumber("leader velocity", arm_.getAngularVelocity());
    SmartDashboard.putNumber("Follower Velocity", arm_.getFollowerAngularVelocity());
  }


  @Override
  public void autonomousInit() {}
  
  @Override
  public void autonomousPeriodic() {}
  
  // Might need this in the future
  // robot_state_.reset(robot_state_.getPosition());
  // robot_state_.update();
  @Override
  public void teleopInit() {
    drive_.setBrakeMode(true);
    arm_.setBrakeMode(true);

    // test().schedule();
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
    driver_controller_.rightBumper().whileTrue(superstructure_.setShooter(-0.5));

    driver_controller_.leftBumper().whileTrue(superstructure_.setIntake(-0.20));

    driver_controller_.pov(0).whileTrue(superstructure_.setIntake(0.3));

    driver_controller_.pov(180).whileTrue(superstructure_.setShooter(0.3));

    driver_controller_.b().whileTrue(superstructure_.shoot());

    ps4_controller_.square().onTrue(test());

    
  }

}
