// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024.auto;

import org.ghrobotics.frc2024.Superstructure;
import org.ghrobotics.frc2024.subsystems.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteTopTaxi extends SequentialCommandGroup {
  PathPlannerPath pickup_top = PathPlannerPath.fromPathFile("Pickup Top");
  public OneNoteTopTaxi(Drive drive, Superstructure superstructure) {
    Command pickupTop = AutoBuilder.followPath(pickup_top);

    addCommands(
      superstructure.setArmPID(20),
      superstructure.setShooter(-0.75),
      pickupTop
      // new ParallelCommandGroup(pickupMiddle, superstructure.setPosition(Position.GROUND_INTAKE)),
    );  }
}
