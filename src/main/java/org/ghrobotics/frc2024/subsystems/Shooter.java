package org.ghrobotics.frc2024.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;


import com.revrobotics.CANSparkMax;

public class Shooter extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax left_leader_; 
    private final CANSparkMax right_leader_; 
    //io
    private final PeriodicIO io_ = new PeriodicIO();

    private final PIDController controller_ = new PIDController(Constants.kP, 0, 0);

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
        right_leader_.follow(left_leader_);
    }
    
    public double getPercent() {
      return io_.demand;
    }

    public void setPercent(double value) {
      double correction = MathUtil.clamp(controller_.calculate(getPercent(), value), -1.0, 1.0);
      io_.demand = correction;
    }

    @Override
    public void periodic(){
        //read inputs
        
        io_.left_leader_supply_current = left_leader_.getOutputCurrent();
        io_.right_leader_supply_current = right_leader_.getOutputCurrent();
        left_leader_.set(io_.demand);
        SmartDashboard.putBoolean("shooterActive", io_.demand>0.2);
    }
    
    // IO
    public static class PeriodicIO {
        //inputs
        double left_leader_supply_current;
        double right_leader_supply_current;
        //ouputs
        double demand;
    }
 
  

    // Constants
    public static class Constants {
        // Motor Controllers
        public static final int kLeftLeaderId = 0;
        public static final int kRightLeaderId = 0;

        // PID
        public static final double kP = 0.8;
    }
}
