// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RebuiltField;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Teleop driving commands: joystick drive, angle-locked drive, snake drive, and alignment. */
public final class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 2.0;
  private static final double ANGLE_KD = 0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier speedMultiplierSupplier,
      BooleanSupplier fieldOrientedSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          double speedMultiplier = speedMultiplierSupplier.getAsDouble();

          Logger.recordOutput("Drive/SpeedMultiplier", speedMultiplier);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * speedMultiplier,
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * speedMultiplier,
              omega * drive.getMaxAngularSpeedRadPerSec() * speedMultiplier);
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

          boolean fieldOriented = fieldOrientedSupplier.getAsBoolean();

          ChassisSpeeds chassisSpeeds = fieldOriented
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation())
              : speeds;

          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for
   * angular control. Possible use cases include snapping to an angle, aiming at
   * a vision target, or controlling absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier,
      DoubleSupplier speedMultiplierSupplier) {

    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP, 0.0, ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(0.5));

    return Commands.run(
        () -> {
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          double omega = angleController.calculate(
              drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

          double speedMultiplier = speedMultiplierSupplier.getAsDouble();

          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * speedMultiplier,
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * speedMultiplier,
              omega * drive.getMaxAngularSpeedRadPerSec() * speedMultiplier);
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /** Rotates the robot in place to face the alliance hub, holding translation at zero. */
  public static Command alignToHub(Drive drive) {
    return alignToHub(drive, () -> 0, () -> 0);
  }

  /** Locks rotation onto the alliance hub while the driver controls translation. */
  public static Command alignToHub(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(
        drive, xSupplier, ySupplier,
        () -> RebuiltField.getTranslationToHub2D().getAngle(),
        () -> 0.75);
  }

  /**
   * Snake drive: the robot automatically rotates so the right-side intake always
   * faces the direction of travel. The driver controls translation with the
   * joystick; heading is managed by a ProfiledPIDController.
   *
   * <p>When the joystick is inside the deadband the last heading goal is held.
   */
  public static Command snakeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier speedMultiplierSupplier) {

    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP, 0.0, ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          // When the joystick is outside the deadband, update the heading goal.
          // The robot's right side is -90° from its front, so for the right side to
          // face the direction of travel the robot heading must be travelDir + 90°.
          if (linearVelocity.getNorm() > 0.0) {
            Rotation2d travelDir =
                new Rotation2d(linearVelocity.getX(), linearVelocity.getY());
            if (isFlipped) {
              travelDir = travelDir.plus(Rotation2d.fromDegrees(180));
            }
            angleController.setGoal(travelDir.plus(Rotation2d.fromDegrees(90)).getRadians());
          }

          // Heading correction is not scaled by speedMultiplier so tracking stays
          // crisp regardless of drive speed mode.
          double omega = angleController.calculate(drive.getRotation().getRadians());
          double speedMultiplier = speedMultiplierSupplier.getAsDouble();

          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * speedMultiplier,
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * speedMultiplier,
              omega * drive.getMaxAngularSpeedRadPerSec());

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive)
        .beforeStarting(
            () -> {
              double heading = drive.getRotation().getRadians();
              angleController.reset(heading);
              angleController.setGoal(heading);
            });
  }

  /**
   * Shake the robot: repeated short motions forward, back, left, right.
   *
   * @param drive           Drive subsystem
   * @param linearFraction  fraction of max linear speed for forward/back (0..1)
   * @param lateralFraction fraction of max linear speed for left/right (0..1)
   * @param pulseSec        duration of each pulse in seconds
   * @param cycles          number of shake cycles (each cycle does forward/back/left/right)
   */
  public static Command shake(
      Drive drive, double linearFraction, double lateralFraction, double pulseSec, int cycles) {
    java.util.List<Command> seq = new java.util.ArrayList<>();

    double max = drive.getMaxLinearSpeedMetersPerSec();
    double fwd = linearFraction * max;
    double lat = lateralFraction * max;

    for (int i = 0; i < cycles; i++) {
      if (linearFraction > 0) {
        seq.add(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(fwd, 0.0, 0.0)), drive)
            .withTimeout(pulseSec));
        seq.add(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-fwd, 0.0, 0.0)), drive)
            .withTimeout(pulseSec));
      }
      if (lateralFraction > 0) {
        seq.add(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, lat, 0.0)), drive)
            .withTimeout(pulseSec));
        seq.add(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, -lat, 0.0)), drive)
            .withTimeout(pulseSec));
      }
    }

    seq.add(Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)), drive));
    return Commands.sequence(seq.toArray(new Command[0]));
  }

  /** Convenience shake command with sane defaults (0.5 fraction, 0.2s pulses, 3 cycles). */
  public static Command shake(Drive drive) {
    return shake(drive, 0.5, 0.5, 0.20, 3);
  }
}
