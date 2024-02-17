package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class Climber extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax left_leader_;
    private final CANSparkMax right_leader_;

    // Encoders
    private final RelativeEncoder left_encoder_;
    private final RelativeEncoder right_encoder_;

    // Control
    private final SimpleMotorFeedforward left_feedforward_;
    private final ProfiledPIDController left_fb_;
    private final SimpleMotorFeedforward right_feedforward_;
    private final ProfiledPIDController right_fb_;

    // IO
    private final PeriodicIO io_ = new PeriodicIO();
    private OutputType left_output_type_ = OutputType.PERCENT;
    private OutputType right_output_type_ = OutputType.PERCENT;

    // Constructor
    public Climber() {
        // Initialize motor controllers.
        left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
        left_leader_.restoreFactoryDefaults();
        left_leader_.setInverted(false);
        left_leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);

        right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
        right_leader_.restoreFactoryDefaults();
        right_leader_.setInverted(true);
        right_leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);

        // Initialize encoders - UPDATE CONVERSION FACTORS!
        left_encoder_ = left_leader_.getEncoder();
        left_encoder_.setPositionConversionFactor(0);
        left_encoder_.setVelocityConversionFactor(0);

        right_encoder_ = right_leader_.getEncoder();
        right_encoder_.setPositionConversionFactor(0);
        right_encoder_.setVelocityConversionFactor(0);


        // Initialize feedforward and feedback.
        left_feedforward_ = new SimpleMotorFeedforward(Constants.kLeftKs, Constants.kLeftKv,
            Constants.kLeftKa);
        left_fb_ = new ProfiledPIDController(Constants.kLeftKp, 0, 0, new TrapezoidProfile.Constraints(
            Constants.kMaxVelocity, Constants.kMaxAcceleration));
        right_feedforward_ = new SimpleMotorFeedforward(Constants.kRightKs, Constants.kRightKv,
            Constants.kRightKa);
        right_fb_ = new ProfiledPIDController(Constants.kRightKp, 0, 0, new TrapezoidProfile.Constraints(
            Constants.kMaxVelocity, Constants.kMaxAcceleration));

        // Configure soft limits.
        left_leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
        left_leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        left_leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxHeightNativeUnits);
        enableSoftLimits(true);

        zero();
    }

    /**
     * This method runs periodically every 20 ms. 
     */
    @Override
    public void periodic() {
        // Read inputs.
        io_.l_position = left_encoder_.getPosition();
        io_.r_position = right_encoder_.getPosition();
        io_.l_velocity = left_encoder_.getVelocity();
        io_.r_velocity = right_encoder_.getVelocity();
        io_.l_current = left_leader_.getOutputCurrent();
        io_.r_current = right_leader_.getOutputCurrent();

        // SmartDashboard.putNumber("L Climb Enc", left_leader_.getSelectedSensorPosition());

        // Zero
        if (io_.wants_zero) {
        io_.wants_zero = false;
        left_encoder_.setPosition(0);
        right_encoder_.setPosition(0);
        }

        // Set motor outputs.
        switch (left_output_type_) {
        case PERCENT:
            // Send the percent output values directly to the motor controller.
            left_leader_.set(io_.l_demand);

        case POSITION:
            // Calculate feedforward value and add to built-in motor controller PID.
            double feedback = left_fb_.calculate(io_.l_position);

            double velocity_setpoint = left_fb_.getSetpoint().velocity;
            double acceleration_setpoint = (velocity_setpoint - io_.l_velocity) / 0.02;
            double feedforward = left_feedforward_.calculate(io_.l_position, velocity_setpoint, acceleration_setpoint);
            
            left_leader_.setVoltage(feedback + feedforward);
            break;
        }

        // Set motor outputs.
        switch (right_output_type_) {
        case PERCENT:
            // Send the percent output values directly to the motor controller.
            right_leader_.set(io_.r_demand);

        case POSITION:
            // Calculate feedforward value and add to built-in motor controller PID.
            double feedback = right_fb_.calculate(io_.r_position);

            double velocity_setpoint = right_fb_.getSetpoint().velocity;
            double acceleration_setpoint = (velocity_setpoint - io_.r_velocity) / 0.02;
            double feedforward = right_feedforward_.calculate(io_.r_position, velocity_setpoint, acceleration_setpoint);
            
            right_leader_.setVoltage(feedback + feedforward);
            break;
        }

    }
    
    public void zero() {
        io_.wants_zero = true;
    }

    /**
     * Enables or disables soft limits for each motor controller. This needs to be done to zero the
     * climber at the start of the match.
     *
     * @param value Whether the soft limits should be enforced.
     */
    public void enableSoftLimits(boolean value) {
        left_leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, value);
        left_leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, value);
        right_leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, value);
        right_leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, value);    
    }

    /**
     * Sets the left arm % output.
     *
     * @param value The left arm % output in [-1, 1].
     */
    public void setLeftPercent(double value) {
        left_output_type_ = OutputType.PERCENT;
        io_.l_demand = value;
    }

    /**
     * Sets the right arm % output.
     *
     * @param value The right arm % output in [-1, 1].
     */
    public void setRightPercent(double value) {
        right_output_type_ = OutputType.PERCENT;
        io_.r_demand = value;
    }

    /**
     * Sets the left arm position.
     *
     * @param value The left arm position in meters.
     */
    public void setLeftPosition(double value) {
        left_output_type_ = OutputType.POSITION;
        io_.l_demand = toCTREPosition(value);
    }

    /**
     * Sets the right arm position.
     *
     * @param value The right arm position in meters.
     */
    public void setRightPosition(double value) {
        right_output_type_ = OutputType.POSITION;
        io_.r_demand = toCTREPosition(value);
    }

    /**
     * Returns the left position of the climber in meters.
     *
     * @return The left position of the climber in meters.
     */
    public double getLeftPosition() {
        return io_.l_position;
    }

    /**
     * Returns the right position of the climber in meters.
     *
     * @return The right position of the climber in meters.
     */
    public double getRightPosition() {
        return io_.r_position;
    }

    /**
     * Returns the left supply current.
     *
     * @return The left supply current.
     */
    public double getLeftSupplyCurrent() {
        return io_.l_current;
    }

    /**
     * Returns the right supply current.
     *
     * @return The right supply current.
     */
    public double getRightSupplyCurrent() {
        return io_.r_current;
    }

    /**
     * Converts position to CTRE native units.
     *
     * @param pos The position in meters.
     * @return The position in native units.
     */
    private static double toCTREPosition(double pos) {
        return pos * Constants.kMaxHeightNativeUnits / Constants.kMaxHeight;
    }

    /**
     * Converts position to CTRE native units / 100 ms.
     *
     * @param vel The velocity in meters per second.
     * @return The velocity in native units / 100 ms.
     */
    private static double toCTREVelocity(double vel) {
        return toCTREPosition(vel) * 10;
    }

    /**
     * Converts CTRE native units to position.
     *
     * @param pos The position in native units.
     * @return The position in meters.
     */
    private static double fromCTREPosition(double pos) {
        return pos * Constants.kMaxHeight / Constants.kMaxHeightNativeUnits;
    }

    /**
     * Converts CTRE native units / 100 ms to velocity.
     *
     * @param vel The velocity in native units / 100 ms.
     * @return The velocity in meters per second.
     */
    private static double fromCTREVelocity(double vel) {
        return fromCTREPosition(vel) / 10;
    }

    public enum OutputType {
        PERCENT, POSITION
    }

    public static class PeriodicIO {
        // Inputs
        double l_position;
        double r_position;

        double l_velocity;
        double r_velocity;

        double l_current;
        double r_current;

        // Outputs
        double l_demand;
        double r_demand;

        boolean wants_zero = false;

    }

        public static class Constants {
        // Motor Controllers
        public static final int kLeftLeaderId = 0;
        public static final int kRightLeaderId = 0;

        // Hardware
        public static final double kMaxHeightNativeUnits = 151543;
        public static final double kMaxHeight = Units.inchesToMeters(25.75);

        // Control

        public static final double kCurrentLimit = 0;

        public static final double kLeftKs = 0;
        public static final double kLeftKv = 0;
        public static final double kLeftKa = 0;
        public static final double kLeftKp = 0.2;

        public static final double kRightKs = 0;
        public static final double kRightKv = 0;
        public static final double kRightKa = 0;
        public static final double kRightKp = 0.2;

        public static final double kMaxVelocity = Units.inchesToMeters(0.1);
        public static final double kMaxAcceleration = Units.inchesToMeters(0.1);
        public static final double kErrorTolerance = Units.inchesToMeters(1.51);
    }
}