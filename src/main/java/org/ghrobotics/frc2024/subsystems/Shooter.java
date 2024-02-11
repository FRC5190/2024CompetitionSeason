package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax left_leader_; 
    private final CANSparkMax right_leader_; 
    
    // Constructor
    public Shooter() {
        // Initialize motor controllers
        left_leader_ = new CANSparkMax(Constants.kLeftLeaderId, MotorType.kBrushless);
        left_leader_.restoreFactoryDefaults();
        left_leader_.setInverted(true);
        left_leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);
        
        right_leader_ = new CANSparkMax(Constants.kRightLeaderId, MotorType.kBrushless);
        right_leader_.restoreFactoryDefaults();
        right_leader_.setInverted(false);
        right_leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
    // IO
    public static class PeriodicIO {
    
    }

    // Constants
    public static class Constants {
        // Motor Controllers
        public static final int kLeftLeaderId = 0;
        public static final int kRightLeaderId = 0;
    }
}
