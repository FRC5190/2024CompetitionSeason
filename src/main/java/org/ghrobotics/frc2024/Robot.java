// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import org.ghrobotics.frc2024.auto.AutoSelector;
// import org.ghrobotics.frc2024.commands.DriveBrakeMode;
// import org.ghrobotics.frc2024.commands.DriveTeleop;
// import org.ghrobotics.frc2024.commands.DriveTowardPosition;
// import org.ghrobotics.frc2024.commands.HomeSuperstructure;
// import org.ghrobotics.frc2024.subsystems.Drivetrain;
import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Climber;
import org.ghrobotics.frc2024.subsystems.Intake;
import org.ghrobotics.frc2024.subsystems.Shooter;
// import org.ghrobotics.frc2024.subsystems.LED;
// import org.ghrobotics.frc2024.subsystems.LED.StandardLEDOutput;
// import org.ghrobotics.frc2024.subsystems.Limelight;
// import org.ghrobotics.frc2024.subsystems.PoseEstimator;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // Subsystems
  private final Arm arm_ = new Arm();
  private final Climber climber_ = new Climber();
  private final Intake intake_ = new Intake();
  private final Shooter shooter_ = new Shooter();

  // Superstructure
  private final Superstructure superstructure_ = new Superstructure(arm_, climber_, intake_, shooter_);

  // Xbox Controllers
  private final CommandXboxController driver_controller_ = new CommandXboxController(0);
  private final CommandXboxController operator_controller_ = new CommandXboxController(1);


  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

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
}
