// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ghrobotics.frc2024.commands.DriveTeleop;
import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Climber;
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
  private final Climber climber_ = new Climber();
  private final Intake intake_ = new Intake();
  private final Shooter shooter_ = new Shooter();
  
  // Robot State
  private final RobotState robot_state_ = new RobotState(drive_);
  
  // Xbox Controller
  private final CommandXboxController driver_controller_ = new CommandXboxController(0);

  // Superstructure
  private final Superstructure superstructure_ = new Superstructure(arm_, climber_, intake_, shooter_);
  
  @Override
  public void robotInit() {
    drive_.setDefaultCommand(new DriveTeleop(drive_, robot_state_, driver_controller_));

    setupTeleopControls();
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    superstructure_.periodic();
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
  }
  
  @Override
  public void teleopPeriodic() {}
  
  @Override
  public void disabledInit() {}
  
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
    driver_controller_.rightBumper().whileTrue(superstructure_.setShooter(1.0));

    driver_controller_.pov(0).whileTrue(superstructure_.setIntake(1.0));
  }

}
