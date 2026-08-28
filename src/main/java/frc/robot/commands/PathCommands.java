// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathCommands {
  /** Creates a new PathCommands. */
  public PathCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static Command goToHubCommand(Drive drive) {
    return goToCoordinate(() -> 2, () -> 4, () -> 0, drive);
  }

  public static Command goToCoordinate(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier angleSupplier, Drive drive) {
    PathConstraints constraints =
        new PathConstraints(4.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return new DeferredCommand(
        () -> {
          Pose2d targetPose = new Pose2d(
              xSupplier.getAsDouble(),
              ySupplier.getAsDouble(),
              Rotation2d.fromDegrees(angleSupplier.getAsDouble()));
          return AutoBuilder.pathfindToPoseFlipped(targetPose, constraints, 0.0);
        },
        Set.of(drive));
  }

  public static Command ClosestTrench(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier opponentSide) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(10, 0.0, 0, new TrapezoidProfile.Constraints(3, 1));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return new DeferredCommand(
        () -> {
          Pose2d robotPose = drive.getPose();

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          if (opponentSide.getAsBoolean()) {
            isFlipped = !isFlipped;
          }

          boolean sideY = false;
          if (robotPose.getY() > 4.00) {
            sideY = true;
          }
          double angle = isFlipped ? 180 : 0;
          double xCoord =
              isFlipped
                  ? Constants.FieldConstants.redLowerTrench.getX()
                  : Constants.FieldConstants.blueLowerTrench.getX();
          double yCoord =
              sideY
                  ? Constants.FieldConstants.redUpperTrench.getY()
                  : Constants.FieldConstants.blueLowerTrench.getY();

          Pose2d targetPose = new Pose2d(xCoord, yCoord, Rotation2d.fromDegrees(angle));

          PathConstraints constraints =
              new PathConstraints(
                  4.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

          return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
        },
        Set.of(drive));
  }
}
