// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeModule;

/** Add your docs here. */
public class IntakeCommands {

  private IntakeCommands() {}

  public static Command raise(IntakeModule intake) {
    return Commands.run(
        () -> {
          intake.raise();
        },
        intake);
  }

  public static Command lower(IntakeModule intake) {
    return Commands.run(
        () -> {
          intake.lower();
        },
        intake);
  }

  public static Command toggle(IntakeModule intake) {
    return Commands.run(
    () -> {
      if (intake.isLowered()) {
        intake.raise();
      } else {
        intake.lower();
      }
    }, intake);
  }
}
