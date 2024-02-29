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
public class TwoNoteBottom extends SequentialCommandGroup {
  PathPlannerPath pickup_bottom = PathPlannerPath.fromPathFile("Pickup Bottom");
  PathPlannerPath shoot_bottom = PathPlannerPath.fromPathFile("Shoot Bottom");

  public TwoNoteBottom(Drive drive, Superstructure superstructure) {
    Command pickupBottom = AutoBuilder.followPath(pickup_bottom);
    Command shootBottom = AutoBuilder.followPath(shoot_bottom);
    
    addCommands(
      superstructure.setPosition(Position.SUBWOOFER),
      superstructure.setShooter(Constants.kSpeakerPercent),
      new ParallelCommandGroup(pickupBottom, superstructure.setPosition(Position.GROUND_INTAKE)),
      shootBottom,
      superstructure.setShooter(Constants.kSpeakerPercent)
    );
  }

  public Pose2d getStartPosition(){
    return pickup_bottom.getStartingDifferentialPose();
  }

  public static class Constants {
    public static double kSpeakerPercent = 1.0;
  }
}