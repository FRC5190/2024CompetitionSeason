package org.ghrobotics.frc2024;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * Swerve Absolute Encoder for CTRE CANCoders.
 */
public class CANCoderSwerve {

  /**
   * CANCoder with WPILib sendable and support.
   */
  public CANcoder encoder;

  /**
   * Initialize the CANCoder on the standard CANBus.
   *
   * @param id CAN ID.
   */
  public CANCoderSwerve(int id) {
    // Empty string uses the default canbus for the system
    encoder = new CANcoder(id);
  }


  /**
   * Reset the encoder to factory defaults.
   */
  public void factoryDefault() {
    encoder.getConfigurator().apply(new CANcoderConfiguration());
  }

  /**
   * Clear sticky faults on the encoder.
   */
  public void clearStickyFaults() {
    encoder.clearStickyFaults();
  }

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  public void configure(boolean inverted) {
    CANcoderConfigurator cfg                       = encoder.getConfigurator();
    MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
    cfg.refresh(magnetSensorConfiguration);
    cfg.apply(magnetSensorConfiguration
      .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withSensorDirection(inverted ? SensorDirectionValue.Clockwise_Positive
          : SensorDirectionValue.CounterClockwise_Positive));
  }

  /**
   * Get the absolute position of the encoder. Sets {@link SwerveAbsoluteEncoder#readingError} on erroneous readings.
   *
   * @return Absolute position in degrees from [0, 360).
   */

  public double getAbsolutePosition() {
    StatusSignal<Double> angle = encoder.getAbsolutePosition();

    return angle.getValue() * 360;
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  
  public Object getAbsoluteEncoder() {
    return encoder;
  }

  /**
   * Sets the Absolute Encoder Offset within the CANcoder's Memory.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point in degrees.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  public boolean setAbsoluteEncoderOffset(double offset) {
    CANcoderConfigurator cfg = encoder.getConfigurator();
    MagnetSensorConfigs magCfg = new MagnetSensorConfigs();
    StatusCode error  = cfg.refresh(magCfg);
    if (error != StatusCode.OK)
    {
      return false;
    }
    error = cfg.apply(magCfg.withMagnetOffset(offset / 360));
    return false;
  }

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  public double getVelocity() {
    return encoder.getVelocity().getValue() * 360;
  }
}
