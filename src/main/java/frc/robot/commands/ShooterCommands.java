// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterStateStore;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.RebuiltField;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterCommands {

  private static final LoggedNetworkNumber flywheelSpeed = new LoggedNetworkNumber("Shooter/Speed", 0);

  public ShooterCommands() {}

  public static Command autoHub(ShooterStateStore state) {
    return Commands.runEnd(
      () -> {
        if (RebuiltField.inNeutralZone()) {
          state.set(RebuiltField.shooterStateForAllianceZone());
        } else {
          state.set(RebuiltField.shooterStateForHub());
        }
      },
      () -> {
        state.reset();
      }
    );
  }

  public static Command manual(ShooterStateStore state, DoubleSupplier hoodChange, DoubleSupplier speedChange) {
    return 
    Commands.sequence(
      Commands.runOnce(() -> { state.returnToLast();}),
      Commands.runEnd(
        () -> {
          state.modifyAngle(-hoodChange.getAsDouble());
          state.modifySpeed(speedChange.getAsDouble());
        },
        () -> {
          state.reset();
        }
      )
    );
  }

  public static Command waitForReady(ShooterSubsystem shooter) {
    return Commands.waitUntil(() -> shooter.getReady());
  }

  public static Command shoot(ShooterSubsystem shooter) {
    return Commands.runEnd(
        () -> {
          shooter.startFeeder();
        },
        () -> {
          shooter.stopFeeder();
        },
        shooter);
  }

  
  public static Command reverseFeeder(ShooterSubsystem shooter) {
    return Commands.runEnd(
        () -> {
          shooter.startReverseFeeder();
        },
        () -> {
          shooter.stopFeeder();
        },
        shooter);
  }


  public static Command rangeForHub(ShooterStateStore state) {
    return Commands.run(
      () -> {
        state.set(RebuiltField.shooterStateForHub());
      }
    );
  }


  public static Command stop(ShooterSubsystem shooter, ShooterStateStore state) {
    return Commands.run(
      () -> {
        state.reset();
        shooter.stopFeeder();
      }
    );
  }

  public static Command lower(ShooterStateStore state) {
    return Commands.runOnce(
      () -> {
        state.set(ShooterState.min);
      }
    );
  }

  
  public static Command startFlywheel(ShooterSubsystem shooter) {
    return Commands.runOnce(() -> shooter.startFlywheel(flywheelSpeed.get()), shooter);
  }

  public static Command stopFlywheel(ShooterSubsystem shooter) {
    return Commands.runOnce(
        () -> {
          shooter.stopFlywheel(true);
        },
        shooter);
  }

  public static Command idleFlywheel(ShooterSubsystem shooter) {
    return Commands.runOnce(
      ()->{
        shooter.startFlywheel(2000);
      },
      shooter
    );
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

  /** Zero the hood using a stall-based zeroing routine (start -> wait until stalled -> set zero). */
  public static Command zeroHood(ShooterSubsystem shooter) {
    return Commands.sequence(
        // Start moving slowly toward the mechanical stop
        Commands.runOnce(() -> shooter.startHoodZeroing(), shooter),
        // Wait until stall condition is detected
        Commands.waitUntil(() -> shooter.isHoodStalled()),
        // Stop and set encoder zero
        Commands.runOnce(() -> shooter.setHoodZero(), shooter));
  }

  public static Command autoShoot(ShooterSubsystem shooter, ShooterStateStore state) {
    return Commands.sequence(
      Commands.runOnce(() -> {
        state.set(RebuiltField.shooterStateForHub());
      }),
      Commands.waitUntil(() -> shooter.getReady()),
      Commands.runOnce(() -> shooter.startFeeder())
    );
  }
}
