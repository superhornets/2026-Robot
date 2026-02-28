// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommands {

  public ShooterCommands() {}

  public static Command update(ShooterSubsystem shooter, Supplier<Pose2d> robotPose) {
    return Commands.run(
        () -> {
          shooter.update(robotPose.get());
        },
        shooter);
  }

  public static Command startFlywheel(ShooterSubsystem shooter) {
    return Commands.runOnce(
        () -> {
          shooter.startFlywheel(6000);
        },
        shooter);
  }

  public static Command stopFlywheel(ShooterSubsystem shooter) {
    return Commands.runOnce(
        () -> {
          shooter.stopFlywheel();
        },
        shooter);
  }

  public static Command startFeeder(ShooterSubsystem shooter) {
    return Commands.runOnce(
        () -> {
          shooter.startFeeder();
        },
        shooter);
  }

  public static Command stopFeeder(ShooterSubsystem shooter) {
    return Commands.runOnce(
        () -> {
          shooter.stopFeeder();
        },
        shooter);
  }

  public static Command startReverse(ShooterSubsystem shooter) {
    return Commands.runOnce(
        () -> {
          shooter.startReverseFeeder();
        },
        shooter);
  }

  public static Command stopReverse(ShooterSubsystem shooter) {
    return Commands.runOnce(
        () -> {
          shooter.stopReverseFeeder();
        },
        shooter);
  }
}
