package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.PipedOutputStream;


public class Arm extends SubsystemBase {

    // Motor Controllers
    private final CANSparkMax leader_; // Left
    private final CANSparkMax follower_; // Right

    // Encoders
    private final RelativeEncoder leader_encoder_;
    private final RelativeEncoder follower_encoder_;

    // Control
    private final ArmFeedforward ff_;
    private final ProfiledPIDController fb_;
    private boolean reset_pid_ = false;
    
    // Simulation
    private final SingleJointedArmSim physics_sim_;
    private final SimDeviceSim leader_sim_;

    // IO
    private final PeriodicIO io_ = new PeriodicIO();
    private OutputType output_type_ = OutputType.PERCENT;
    
        
    // Constructor
    public Arm() {
        // Initialize motor controllers
        leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
        leader_.restoreFactoryDefaults();
        leader_.setInverted(true);
        leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);
        
        follower_ = new CANSparkMax(Constants.kFollowerId, MotorType.kBrushless);
        follower_.restoreFactoryDefaults();
        follower_.setInverted(false);
        follower_.setIdleMode(CANSparkMax.IdleMode.kCoast);

        // Initialize encoders
        leader_encoder_ = leader_.getEncoder();
        leader_encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio);
        leader_encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kGearRatio / 60);

        follower_encoder_ = follower_.getEncoder();
        follower_encoder_.setPositionConversionFactor(2 * Math.PI / Constants.kGearRatio);
        follower_encoder_.setVelocityConversionFactor(2 * Math.PI / Constants.kGearRatio / 60);

        // Initialize control
        ff_ = new ArmFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);
        fb_ = new ProfiledPIDController(Constants.kP, 0, 0, new TrapezoidProfile.Constraints(
            Constants.kMaxVelocity, Constants.kMaxAcceleration));

        // Safety features
        leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
        leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.kMinAngle);
        leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxAngle);

        // Initialize simulation
        physics_sim_ = new SingleJointedArmSim(
            LinearSystemId.identifyPositionSystem(Constants.kV, Constants.kA),
            DCMotor.getNEO(2), Constants.kGearRatio, Constants.kArmLength, Constants.kMinAngle,
            Constants.kMaxAngle, false, 0
            );
        leader_sim_ = new SimDeviceSim("SPARK MAX [" + Constants.kLeaderId + "]");

        // Safety features
        leader_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
        leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.kMinAngle);
        leader_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxAngle);
        
        follower_.setSmartCurrentLimit((int) Constants.kCurrentLimit);
        follower_.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.kMinAngle);
        follower_.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) Constants.kMaxAngle);

        // Reset encoder
        zero();
        physics_sim_.setState(VecBuilder.fill(Constants.kMaxAngle, 0));
    }

    @Override
    public void periodic() {
        // Read inputs
        io_.angle = leader_encoder_.getPosition();
        io_.angular_velocity = leader_encoder_.getVelocity();
        io_.current = leader_.getOutputCurrent();

        io_.angle = follower_encoder_.getPosition();
        io_.angular_velocity = follower_encoder_.getVelocity();
        io_.current = follower_.getOutputCurrent();

        if (io_.wants_zero) {
        io_.wants_zero = false;
            leader_encoder_.setPosition(Constants.kMaxAngle);
            follower_encoder_.setPosition(Constants.kMaxAngle);
        }

        // Reset controller if we have to
        if (reset_pid_) {
            reset_pid_ = false;
            fb_.reset(io_.angle, io_.angular_velocity);
        }

        // Write outputs
        switch (output_type_) {
        case PERCENT:
            leader_.set(io_.demand);
            follower_.set(io_.demand);

            // Set simulated inputs
            if (RobotBase.isSimulation())
            leader_sim_.getDouble("Applied Output").set(io_.demand * 12);

            break;
        case ANGLE:
            double feedback = fb_.calculate(io_.angle);

            double velocity_setpoint = fb_.getSetpoint().velocity;
            double acceleration_setpoint = (velocity_setpoint - io_.angular_velocity) / 0.02;
            double feedforward = ff_.calculate(io_.angle, velocity_setpoint, acceleration_setpoint);

            leader_.setVoltage(feedback + feedforward);
            follower_.setVoltage(feedback + feedforward);
            break;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update physics sim with inputs
        double voltage = leader_.getAppliedOutput();
        if (output_type_ == OutputType.ANGLE)
        voltage -= Constants.kG * Math.cos(io_.angle);
        physics_sim_.setInputVoltage(voltage);

        // Update physics sim forward in time
        physics_sim_.update(0.02);

        // Update encoder values
        leader_sim_.getDouble("Position").set(physics_sim_.getAngleRads());
        leader_sim_.getDouble("Velocity").set(physics_sim_.getVelocityRadPerSec());
    }

    public void setPercent(double percent) {
        output_type_ = OutputType.PERCENT;
        io_.demand = percent;
        reset_pid_ = true;
    }

    public void setAngle(double angle) {
        output_type_ = OutputType.ANGLE;
        fb_.setGoal(angle);
        reset_pid_ = true;
    }

    public void enableSoftLimits(boolean value) {
        leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, value);
        leader_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, value);
        follower_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, value);
        follower_.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, value);    
    }

    public void zero() {
        io_.wants_zero = true;
    }

    // Position Getter
    public double getAngle() {
        return io_.angle;
    }

    public double getAngularVelocity() {
        return io_.angular_velocity;
    }

    public double getAngularVelocitySetpoint() {
        return fb_.getSetpoint().velocity;
    }

    public double getCurrent() {
        return io_.current;
    }

    // Output Type
    private enum OutputType {
        PERCENT, ANGLE
    }

    //IO
    private static class PeriodicIO {
        //Inputs
        double angle;
        double angular_velocity;
        double current;

        //Outputs
        boolean wants_zero;
        double demand;
    }
        

    // Constants (TO UPDATE)
    public static class Constants {
        // Motor Controllers
        public static final int kLeaderId = 0;
        public static final int kFollowerId = 0;

        // Physical Constants
        public static final double kGearRatio = 0;
        public static final double kMinAngle = Math.toRadians(0);
        public static final double kMaxAngle = Math.toRadians(0);
        public static final double kArmLength = 0;

        // Feedforward
        public static final double kA = 0; // volts * seconds^2 / radians
        public static final double kG = 0; // volts
        public static final double kS = 0; // volts
        public static final double kV = 0; // volts * seconds/radians

        // Current Limit
        public static final double kCurrentLimit = 0;

        // Control
        public static double kMaxVelocity = 0;
        public static double kMaxAcceleration = 0;
        public static double kP = 0;
    }
}

