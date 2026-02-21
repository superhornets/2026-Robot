// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommands {

  public ShooterCommands() {}

  public static Command runFlywheel(ShooterSubsystem shooter) {
    return Commands.startEnd(
        () -> {
          shooter.startFlywheel(2000);
        },
        () -> {
          shooter.stopFlywheel();
        },
        shooter);
  }

  public static Command runFeeder(ShooterSubsystem shooter) {
    return Commands.startEnd(
        () -> {
          shooter.startFeeder();
        },
        () -> {
          shooter.stopFeeder();
        },
        shooter);
  }
}
