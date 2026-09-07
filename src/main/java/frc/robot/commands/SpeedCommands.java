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
    var slow = Commands.startEnd(speed::setSlow, speed::reset);
    slow.setName("Slow Command");
    return slow;
  }

  public static Command fastCommand(SpeedSupplier speed) {
    var fast = Commands.startEnd(speed::setFast, speed::reset);
    fast.setName("Fast Command");
    return fast;
  }

  public static Command intakeCommand(SpeedSupplier speed) {
    var intake = Commands.startEnd(speed::setIntake, speed::reset);
    intake.setName("Intake Command");
    return intake;
  }
}
