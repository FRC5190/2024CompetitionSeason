// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024.auto;

import org.ghrobotics.frc2024.Superstructure;
import org.ghrobotics.frc2024.Superstructure.Position;
import org.ghrobotics.frc2024.subsystems.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteMiddleTaxi extends SequentialCommandGroup {
  PathPlannerPath pickup_middle = PathPlannerPath.fromPathFile("Pickup Middle");

  public OneNoteMiddleTaxi(Drive drive, Superstructure superstructure) {
    Command pickupMiddle = AutoBuilder.followPath(pickup_middle);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      superstructure.setArmPID(-30),
      //new ParallelDeadlineGroup(superstructure.setShooter(-0.75), new WaitCommand(0.5)),
      superstructure.setShooter(-0.75),
      pickupMiddle
      // new ParallelCommandGroup(pickupMiddle, superstructure.setPosition(Position.GROUND_INTAKE)),
    );
  }
}
