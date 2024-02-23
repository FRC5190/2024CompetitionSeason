package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax leader_;
  private final PeriodicIO io_ = new PeriodicIO();

  private final PIDController controller_ = new PIDController(Constants.kP, 0, 0);

  // Constructor
  public Intake() {
    // Initialize motor controllers
    leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
    leader_.restoreFactoryDefaults();
    leader_.setInverted(true);
    leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
  
  public void periodic() {
    // Read inputs.
    io_.current = leader_.getOutputCurrent();
    leader_.set(io_.demand);
  }

  public double getPercent() {
    return io_.demand;
  }

  public void setPercent(double value) {
    double correction = MathUtil.clamp(controller_.calculate(getPercent(), value), -1.0, 1.0);
    io_.demand = correction;
  }

  // IO
  public static class PeriodicIO {
    // Input
    double current;

    // Output
    double demand;
  }

  // Constants
  public static class Constants {
    // Motor Controllers
    public static final int kLeaderId = 0;

    // PID Constants
    public static final double kP = 0.8;
  }
}
