package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax leader_left_;
  private final CANSparkMax leader_right_;
  private final PeriodicIO io_ = new PeriodicIO();

  private final PIDController controller_ = new PIDController(Constants.kP, 0, 0);

  // Constructor
  public Intake() {
    // Initialize motor controllers
    leader_left_ = new CANSparkMax(Constants.kLeaderLeftId, MotorType.kBrushless);
    leader_left_.restoreFactoryDefaults();
    leader_left_.setInverted(true);
    leader_left_.setIdleMode(CANSparkMax.IdleMode.kCoast);


    leader_right_ = new CANSparkMax(Constants.kLeaderRightId, MotorType.kBrushless);
    leader_right_.restoreFactoryDefaults();
    leader_right_.setInverted(true);
    leader_right_.setIdleMode(CANSparkMax.IdleMode.kCoast);
    leader_right_.follow(leader_left_);

    // Safety
    leader_left_.setSmartCurrentLimit(30);
    leader_right_.setSmartCurrentLimit(30);
    
  }
  
  public void periodic() {
    // Read inputs.
    io_.current_left_ = leader_left_.getOutputCurrent();
    io_.current_right_ = leader_right_.getOutputCurrent();
    SmartDashboard.putNumber("Left Intake current", io_.current_left_);
    SmartDashboard.putNumber("Right Intake current", io_.current_right_);
    if(io_.left_demand == 0) {
      leader_left_.set(io_.left_demand);
    } else {
      leader_left_.set(io_.left_demand - 0.3);
    }
    
    leader_right_.set(-io_.right_demand);
  }

  // public double getPercent() {
  //   return io_.demand;
  // }

  // public void setPercent(double value) {
  //   double correction = MathUtil.clamp(controller_.calculate(getPercent(), value), -1.0, 1.0);
  //   io_.left_demand = correction;
  //   io_.right_demand = correction;
  // }

  public void setPercent(double value) {
    io_.left_demand = value + 0.2;
    io_.right_demand = value;
  }
  public void stopMotor() {
    io_.left_demand = 0;
    io_.right_demand = 0;
  }

  public double getLeftOutputCurrent() {
    return io_.current_left_;
  }

  public double getRightOutputCurrent() {
    return io_.current_right_;
  }

  // IO
  public static class PeriodicIO {
    // Input
    double current_left_;
    double current_right_;

    // Output
    double left_demand;
    double right_demand;
  }

  // Constants
  public static class Constants {
    // Motor Controllers
    public static final int kLeaderLeftId = 11;
    public static final int kLeaderRightId = 12;

    // PID Constants
    public static final double kP = 0.8;
  }
}
