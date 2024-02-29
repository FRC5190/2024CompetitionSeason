package org.ghrobotics.frc2024;

import java.util.ArrayList;
import java.util.List;

import org.ghrobotics.frc2024.subsystems.Arm;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {

  // Periodic
  private final List<Runnable> periodic_registry_ = new ArrayList<>();

  private final Field2d field_;

  private final Mechanism2d superstructure_;


  public Telemetry(RobotState robot_state, Arm arm) {
    ShuffleboardTab tab_ = Shuffleboard.getTab("2024");

    field_ = new Field2d();
        tab_.add("Field", field_);
        if (Robot.isReal())
            periodic_registry_.add(() -> field_.setRobotPose(robot_state.getPosition()));
        if(!Robot.isReal())
            field_.setRobotPose(null);

    superstructure_ = new Mechanism2d(1.2, 1.2);

    tab_.add("Superstructure", superstructure_);

    superstructure_.getRoot("Structure Root", 0.33, 0.1).append(
      new MechanismLigament2d("Structure", 0.5, 0));
    MechanismLigament2d arm_out = superstructure_.getRoot("Carriage Root", 0.80, 0.15).append(
      new MechanismLigament2d("Arm", 0.15, 0, 3, new Color8Bit(Color.kOrangeRed)));

    periodic_registry_.add(() -> arm_out.setAngle(Math.toDegrees(arm.getAngle())));
  }

  public void periodic() {
    for (Runnable fn : periodic_registry_)
      fn.run();
    // System.out.println(periodic_registry_);
  }
}
