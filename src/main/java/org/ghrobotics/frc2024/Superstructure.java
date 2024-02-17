package org.ghrobotics.frc2024;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import java.util.function.DoubleSupplier;

import org.ghrobotics.frc2024.commands.ArmToPosition;
import org.ghrobotics.frc2024.commands.ClimbReset;
import org.ghrobotics.frc2024.commands.ClimbToPosition;
import org.ghrobotics.frc2024.commands.ClimbTeleop;

import org.ghrobotics.frc2024.subsystems.Arm;
import org.ghrobotics.frc2024.subsystems.Climber;
import org.ghrobotics.frc2024.subsystems.Intake;
import org.ghrobotics.frc2024.subsystems.Shooter;



public class Superstructure {
    // Subsystems
    private final Arm arm_;
    private final Climber climber_;
    private final Intake intake_;
    private final Shooter shooter_;

    //Store Position
    public String state = "STOW";

    // Constructor
    public Superstructure(Arm arm, Climber climber, Intake intake, Shooter shooter) {
        arm_ = arm;
        climber_ = climber;
        intake_ = intake;
        shooter_ = shooter;
    }

    // Position Setter
    public Command setPosition(Position pos) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> this.state = pos.posname),
            new ParallelCommandGroup(
                new ArmToPosition(arm_, pos.angle)
            ).withTimeout(6)
        );
    }

    //GetPosition of Superstructure
    public String getState() {
        //System.out.println(state);
        return state;
    }

    public enum Position {
        STOW(0, "STOW"),
        SUBWOOFER(0, "SUBWOOFER"),
        AMP(0, "AMP"),
        GROUND_INTAKE(0, "GROUND_INTAKE"),
        SOURCE_INTAKE(0, "SOURCE_INTAKE");
        
        final double angle;
        final String posname;

        Position(double angle_deg, String name) {
            this.angle = Math.toRadians(angle_deg);
            this.posname = name;
        }

    }

}
