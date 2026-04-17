// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeModule;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class IntakeCommands {

  private IntakeCommands() {}

  public static Command raiseAll(IntakeModule intake) {
    return Commands.run(
        () -> {
          intake.raise();
        },
        intake);
  }



  public static Command lowerRight(IntakeModule intake) {
    return Commands.startEnd(
        () -> {
          intake.lower();
        },
        () -> {
          intake.raise();
        },
        intake);
  }

  public static Command toggle(IntakeSubsystem intake) {
    return Commands.run(
    () -> {
      if (intake.isLowered()) {
        intake.raiseAll();
      } else {
        intake.lowerRight();
      }
    }, intake);
  }
}
