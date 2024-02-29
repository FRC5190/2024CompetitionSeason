// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ghrobotics.frc2024.auto;

import org.ghrobotics.frc2024.Superstructure;
import org.ghrobotics.frc2024.Superstructure.Position;
import org.ghrobotics.frc2024.subsystems.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteTop extends SequentialCommandGroup {
  PathPlannerPath pickup_top = PathPlannerPath.fromPathFile("Pickup Top");
  PathPlannerPath shoot_top = PathPlannerPath.fromPathFile("Shoot Top");
  
  public TwoNoteTop(Drive drive, Superstructure superstructure) {
    Command pickupTop = AutoBuilder.followPath(pickup_top);
    Command shootTop = AutoBuilder.followPath(shoot_top);
    
    addCommands(
      superstructure.setPosition(Position.SUBWOOFER),
      superstructure.setShooter(Constants.kSpeakerPercent),
      new ParallelCommandGroup(pickupTop, superstructure.setPosition(Position.GROUND_INTAKE)),
      shootTop,
      superstructure.setShooter(Constants.kSpeakerPercent)
    );
  }

  public Pose2d getStartPosition(){
    return pickup_top.getStartingDifferentialPose();
  }

  public static class Constants {
    public static double kSpeakerPercent = 1.0;
  }
}
