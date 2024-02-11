package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax leader_; // Left
    
    // Constructor
    public Intake() {
        // Initialize motor controllers
        leader_ = new CANSparkMax(Constants.kLeaderId, MotorType.kBrushless);
        leader_.restoreFactoryDefaults();
        leader_.setInverted(true);
        leader_.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    // IO
    public static class PeriodicIO {
    
    }

    // Constants
    public static class Constants {
        // Motor Controllers
        public static final int kLeaderId = 0;
        public static final int kFollowerId = 0;
    }
}
