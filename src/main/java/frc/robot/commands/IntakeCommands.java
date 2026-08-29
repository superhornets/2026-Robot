// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class IntakeCommands {

  private IntakeCommands() {}

  public static Command raise(IntakeSubsystem intake) {
    return Commands.run(
        () -> {
          intake.raise();
        },
        intake);
  }

  public static Command lower(IntakeSubsystem intake) {
    return Commands.run(
        () -> {
          intake.lower();
        },
        intake);
  }

  public static Command toggle(IntakeSubsystem intake) {
    return Commands.run(
    () -> {
      if (intake.isLowered()) {
        intake.raise();
      } else {
        intake.lower();
      }
    }, intake);
  }

  public static Command intakeAgitate(IntakeSubsystem intake) {
    return Commands.repeatingSequence(
      Commands.waitSeconds(0.5),
      Commands.runOnce(
        () -> {
          intake.raise();
          intake.stopRoller();
        }, intake
      ),
      Commands.waitSeconds(0.5),
      Commands.runOnce(
        () -> {
          intake.lower();
          intake.startRoller(false);
        }, intake)
    );
  }
}
//     return Commands.runEnd(
//       () -> {
//         if (intake.isAtSetpoint()) {
//           if (intake.isLowered()) {
//             intake.raiseHalf();
//           } else {
//             intake.lower();
//           }
//         }
//       }, 
//       () -> {
//         intake.lower();
//       }, intake
//     );
//   } }
