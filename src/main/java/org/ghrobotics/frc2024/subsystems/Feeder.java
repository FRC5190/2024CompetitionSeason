package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase{
  private final CANSparkMax leader_;

  private final PeriodicIO io_ = new PeriodicIO();

  public Feeder() {
    leader_ = new CANSparkMax(Constants.kFeederId, MotorType.kBrushless);
    leader_.restoreFactoryDefaults();
    leader_.setInverted(false);
    leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    leader_.set(io_.demand);
  }

  public void setPercent(double value) {
    io_.demand = value;
  }

  public void stopMotor() {
    leader_.set(0);
  }

  public double getPercent() {
    return leader_.get();
  }

  // IO
  public static class PeriodicIO {
    public double demand;
  }

  public static class Constants {
    public static final int kFeederId = 15;
  }
}
