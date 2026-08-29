// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.SpeedSupplier;

/** Commands that adjust the robot's drive speed mode via a {@link SpeedSupplier}. */
public final class SpeedCommands {
  private SpeedCommands() {}

  public static Command slowCommand(SpeedSupplier speed) {
    return Commands.startEnd(speed::setSlow, speed::reset);
  }

  public static Command fastCommand(SpeedSupplier speed) {
    return Commands.startEnd(speed::setFast, speed::reset);
  }

  public static Command intakeCommand(SpeedSupplier speed) {
    return Commands.startEnd(speed::setIntake, speed::reset);
  }
}
