// package org.ghrobotics.frc2024.auto;

// import org.ghrobotics.frc2024.RobotState;
// import org.ghrobotics.frc2024.Superstructure;
// import org.ghrobotics.frc2024.commands.ArmPID;
// import org.ghrobotics.frc2024.subsystems.Arm;
// import org.ghrobotics.frc2024.subsystems.Drive;
// import org.ghrobotics.frc2024.subsystems.Feeder;
// import org.ghrobotics.frc2024.subsystems.Intake;
// import org.ghrobotics.frc2024.subsystems.Shooter;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// public class FourNoteAuto extends SequentialCommandGroup{
  
//   PathPlannerPath middle_middle_intake_path = PathPlannerPath.fromPathFile("middle_middle_intake");
//   PathPlannerPath middle_middle_shoot_path = PathPlannerPath.fromPathFile("middle_middle_shoot");
//   PathPlannerPath middle_right_intake_path = PathPlannerPath.fromPathFile("middle_right_intake");
//   PathPlannerPath middle_left_intake_path = PathPlannerPath.fromPathFile("middle_left_intake");
//   PathPlannerPath left_shoot_path = PathPlannerPath.fromPathFile("left_shoot");

//   // Four Note Auto Path
//   PathPlannerPath left_intake_path = PathPlannerPath.fromPathFile("left_intake");
//   PathPlannerPath middle_intake_path = PathPlannerPath.fromPathFile("middle_intake");
//   PathPlannerPath right_intake_path = PathPlannerPath.fromPathFile("right_intake");

//   PathPlannerPath left_one_intake_path = PathPlannerPath.fromPathFile("left_one_intake");

//   // Constructor
//   public FourNoteAuto(Drive drive, RobotState robot_state, Superstructure superstructure, Arm arm, Intake intake, Shooter shooter, Feeder feeder) {
//     addCommands(
//       // Part 1
//       new ParallelCommandGroup(
//         // Rev shooter, follow path to intake
//         new InstantCommand(() -> shooter.setPercent(0.75)),
//         AutoBuilder.followPath(left_intake_path),
//         new SequentialCommandGroup(
//           new ArmPID(arm, 25),
//           new WaitCommand(0.5),
//           new InstantCommand(() -> intake.setPercent(0.55)),
//           new InstantCommand(() -> feeder.setPercent(0.5)),
//           new WaitCommand(0.2),
//           new ArmPID(arm, 2),
//           new InstantCommand(() -> intake.setPercent(0.55)),
//           new InstantCommand(() -> feeder.stopMotor())
//         )
//       ),
//       // Part 2
//       new ParallelCommandGroup(
//         // Rev shooter, follow path to shoot
//         // new InstantCommand(() -> shooter_.setPercent(-0.75)),
//         AutoBuilder.followPath(middle_intake_path),
//         new SequentialCommandGroup(
//           new ArmPID(arm, 29),
//           new WaitCommand(0.5),
//           new InstantCommand(() -> intake.setPercent(0.55)),
//           new InstantCommand(() -> feeder.setPercent(0.5)),
//           new WaitCommand(0.5),
//           new ArmPID(arm, 2),
//           new InstantCommand(() -> intake.setPercent(0.55)),
//           new InstantCommand(() -> feeder.stopMotor())
//         )
//       ),
//       // Part 3
//       new ParallelCommandGroup(
//         AutoBuilder.followPath(right_intake_path),
//         new SequentialCommandGroup(
//           new ArmPID(arm, 29),
//           new WaitCommand(0.5),
//           new InstantCommand(() -> feeder.setPercent(0.5)),
//           new InstantCommand(() -> intake.setPercent(0.55)),
//           new WaitCommand(0.5),
//           new ArmPID(arm, 2).withTimeout(1.5),
//           new InstantCommand(() -> feeder.stopMotor()),
//           new InstantCommand(() -> intake.setPercent(0.55))
          
//         )
//       ),
//       // Part 4
//       new ArmPID(arm, 32.85),
//       new WaitCommand(0.2),
//       new InstantCommand(() -> feeder.setPercent(0.75)),
//       new InstantCommand(() -> intake.setPercent(0.5)),
//       new WaitCommand(1.0),
//       new ArmPID(arm, 2)
//     );
//   }
// }
