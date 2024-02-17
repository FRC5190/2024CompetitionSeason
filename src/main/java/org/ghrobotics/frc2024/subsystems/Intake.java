package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax leader_; // Left
    private final PeriodicIO io_ = new PeriodicIO();
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
        io_.leader_supply_current = leader_.getOutputCurrent();
        leader_.set(io_.leader_bridge_demand);
    }
 
    //get, set % output on intake. parameter is percent output [-1, 1]
    public double getPercent() {
        return io_.leader_bridge_demand;
    }
    public void setPercent(double value) {
        io_.leader_bridge_demand = value;
    }
    //Returns the % output of the intake and bridge.
 
    // IO
    public static class PeriodicIO {
        //input
        double leader_supply_current;
        //output
        double leader_bridge_demand;
    }

    // Constants
    public static class Constants {
        // Motor Controllers
        public static final int kLeaderId = 0;
        public static final int kFollowerId = 0;
    }
}
