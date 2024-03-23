package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Shooter extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax left_leader_; 
  private final CANSparkMax right_leader_; 

  // Encoders
  private final RelativeEncoder left_encoder_;
  private final RelativeEncoder right_encoder_;

  // IO
  private final PeriodicIO io_ = new PeriodicIO();
  private OutputType output_type_ = OutputType.PERCENT;

  private final PIDController pid_ = new PIDController(Constants.kP, 0, 0);

  // Constructor
  public Shooter() {
    // Initialize motor controllers
    left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
    left_leader_.restoreFactoryDefaults();
    left_leader_.setInverted(false);
    left_leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);
    left_leader_.enableVoltageCompensation(12.0);
    
    right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
    right_leader_.restoreFactoryDefaults();
    right_leader_.setInverted(false);
    right_leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);
    right_leader_.enableVoltageCompensation(12.0);

    left_encoder_ = left_leader_.getEncoder();
    left_encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kLeftGearRatio);
    left_encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kLeftGearRatio / 60);

    right_encoder_ = right_leader_.getEncoder();
    right_encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kRightGearRatio);
    right_encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kRightGearRatio / 60);

    // Safety
    left_leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
    right_leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);

  }

  @Override
  public void periodic(){
    // Read inputs
    io_.left_velocity = left_encoder_.getVelocity();
    io_.right_velocity = right_encoder_.getVelocity();

    io_.left_leader_supply_current = left_leader_.getOutputCurrent();
    io_.right_leader_supply_current = right_leader_.getOutputCurrent();

    // Update SmartDashboard
    SmartDashboard.putNumber("Shooter L Velocity", io_.left_velocity);
    SmartDashboard.putNumber("Shooter R Velocity", io_.right_velocity);

    switch (output_type_) {
      case PERCENT:
        if(io_.demand <= 0.1) {
          left_leader_.set(io_.demand);
          right_leader_.set(io_.demand);
        } else {
          left_leader_.set(io_.demand);
          right_leader_.set(io_.demand - 0.15);
        }
        break;
      case VELOCITY:
        double left_feedback = pid_.calculate(io_.left_velocity);
        double right_feedback = pid_.calculate(io_.right_velocity);

        left_leader_.set(left_feedback);
        right_leader_.set(right_feedback);
        break;
    }
  }

  public double getPercent() {
    return io_.demand;
  }

  /**
   * Sets the shooter velocity.
   *
   * @param value The velocity in rotations per second.
   */
  public void setVelocity(double value) {
    output_type_ = OutputType.VELOCITY;
    // Convert to radians per second to rotations per minute
    value = value * 60 / (2 * Math.PI);
    pid_.setSetpoint(value);
  }

  public void setPercent(double value) {
    io_.demand = value;
  }

  public void stopMotor() {
    io_.demand = 0;
  }

  public enum OutputType {
    PERCENT, VELOCITY
  }
  
  // IO
  public static class PeriodicIO {
    // Inputs
    double left_velocity;
    double right_velocity;
    double left_leader_supply_current;
    double right_leader_supply_current;

    // Outputs
    double demand;
  }

  // Constants
  public static class Constants {
    // Motor Controllers
    public static final int kLeftLeaderId = 13;
    public static final int kRightLeaderId = 14;

    // Gear Ratio
    public static final double kLeftGearRatio = 1;
    public static final double kRightGearRatio = 1;

    // PID
    public static final double kP = 2.8;

    // Current Limit
    public static final double kCurrentLimit = 30;
  }
}
