// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class IntakeCommands {

  private IntakeCommands() {}

  public static Command lowerLeft(IntakeSubsystem intake) {
    return Commands.run(
        () -> {
          intake.lowerLeft();
        },
        intake);
  }

  public static Command lowerRight(IntakeSubsystem intake) {
    return Commands.run(
        () -> {
          intake.lowerRight();
        },
        intake);
  }
}
