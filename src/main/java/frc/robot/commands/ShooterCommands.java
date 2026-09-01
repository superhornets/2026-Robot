// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterStateStore;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.RebuiltField;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class ShooterCommands {

  private ShooterCommands() {}

  /** Sets the shooter to the idle state, which spins the flywheel at a low speed for fast spin-up. */
  public static Command warmup(ShooterStateStore state) {
    return Commands.runOnce(() -> state.idle());
  }

  /** Continuously primes the shooter for the hub, switching to the alliance-zone preset while in the neutral zone. */
  public static Command primeShooter(ShooterStateStore state) {
    return Commands.runEnd(
        () -> {
          if (RebuiltField.inNeutralZone()) {
            state.set(RebuiltField.shooterStateForAllianceZone());
          } else {
            state.set(RebuiltField.shooterStateForHub());
          }
        },
        () -> state.idle());
  }

  /** Incrementally adjusts hood angle and flywheel speed from joystick axes. Restores last manual state on start, resets on cancel. */
  public static Command adjustAim(ShooterStateStore state, DoubleSupplier hoodChange, DoubleSupplier speedChange) {
    return Commands.sequence(
        Commands.runOnce(() -> state.returnToLast()),
        Commands.runEnd(
            () -> {
              state.modifyAngle(-hoodChange.getAsDouble());
              state.modifySpeed(speedChange.getAsDouble());
            },
            () -> state.idle()));
  }

  /** Runs the feeder and spindexer while held, stopping both on release. */
  public static Command feed(ShooterSubsystem shooter) {
    return Commands.runEnd(
        () -> shooter.startFeeder(),
        () -> shooter.stopFeeder(),
        shooter);
  }

  /** Reverses the spindexer to clear a jam. */
  public static Command unjam(ShooterSubsystem shooter) {
    return Commands.runEnd(
        () -> shooter.startReverseFeeder(),
        () -> shooter.stopFeeder(),
        shooter);
  }

  /** Fires the feeder once — intended for use inside autonomous sequences. */
  public static Command triggerFeed(ShooterSubsystem shooter) {
    return Commands.runOnce(() -> shooter.startFeeder(), shooter);
  }

  /**
   * Idles the shooter: reduces flywheel to {@link ShooterConstants#kFlywheelIdleSpeed} for fast spin-up
   * and stops the feeder. Requires {@link ShooterSubsystem} so that scheduling this command interrupts
   * any active {@link #startAimingAtHub} command.
   */
  public static Command idle(ShooterSubsystem shooter, ShooterStateStore state) {
    return Commands.runOnce(
        () -> {
          state.idle();
          shooter.stopFeeder();
        },
        shooter);
  }

  /** Waits until {@code ready} is true, then runs the feeder until interrupted. */
  public static Command feedWhenReady(ShooterSubsystem shooter, Trigger ready) {
    return Commands.sequence(
        Commands.waitUntil(ready),
        feed(shooter)
    );
  }

  /**
   * Locks the robot onto the hub (no driver translation), aims the shooter, and fires once
   * {@code readyToFire} is true. On termination, idles the shooter and stops the feeder.
   */
  public static Command autoShoot(Drive drive, ShooterSubsystem shooter, ShooterStateStore state, Trigger readyToFire) {
    return autoShoot(drive, shooter, state, readyToFire, () -> 0, () -> 0);
  }

  /**
   * Rotates the robot to face the hub while the driver controls translation, aims the shooter,
   * and fires once {@code readyToFire} is true.
   * On termination, idles the shooter and stops the feeder.
   */
  public static Command autoShoot(Drive drive, ShooterSubsystem shooter, ShooterStateStore state, Trigger readyToFire, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.parallel(
        DriveCommands.alignToHub(drive, xSupplier, ySupplier),
        primeShooter(state),
        feedWhenReady(shooter, readyToFire)
    );
  }

  /**
   * Aims the shooter and fires once {@code readyToFire} is true, without requiring the drive
   * subsystem — rotation is left entirely to the driver or autonomous path.
   * On termination, idles the shooter and stops the feeder.
   */
  public static Command autoShoot(ShooterSubsystem shooter, ShooterStateStore state, Trigger readyToFire) {
    return Commands.parallel(
        primeShooter(state),
        feedWhenReady(shooter, readyToFire)
    );
  }

  /** Homes the hood by driving it to the hard stop, detecting a stall, then zeroing the encoder. */
  public static Command homeHood(ShooterSubsystem shooter) {
    return Commands.sequence(
        Commands.runOnce(() -> shooter.startHoodZeroing(), shooter),
        Commands.waitUntil(() -> shooter.isHoodStalled()),
        Commands.runOnce(() -> shooter.setHoodZero(), shooter));
  }
}
