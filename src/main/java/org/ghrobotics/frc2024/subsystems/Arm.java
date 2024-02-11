package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.StateSpaceUtil;
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
    // private final ArmFeedforward ff_;
    // private final ProfiledPIDController fb_;
    // private boolean reset_pid_ = false;

    
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
    }
   
   
    // IO
    public static class PeriodicIO {
    
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

    }
}

