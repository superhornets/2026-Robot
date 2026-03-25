// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

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

  public static Command goToHubCommand() {
    return goToCoordinate(() -> 2, () -> 4, () -> 0);
  }

  public static Command goToCoordinate(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier angleSupplier) {

    Pose2d targetPose = new Pose2d(xSupplier.getAsDouble(), ySupplier.getAsDouble(), Rotation2d.fromDegrees(angleSupplier.getAsDouble()));
    
    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(35.0, 14.0, Units.degreesToRadians(900), Units.degreesToRadians(720));

    Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose, constraints, 0.0);
    return pathfindingCommand;
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

          boolean SideY = false;
          if (robotPose.getY() > 4.00) {
            SideY = true;
          }
          double angle = isFlipped ? 180 : 0;
          double xCoord =
              isFlipped
                  ? Constants.FieldConstants.redLowerTrench.getX()
                  : Constants.FieldConstants.blueLowerTrench.getX();
          double yCoord =
              SideY
                  ? Constants.FieldConstants.redUpperTrench.getY()
                  : Constants.FieldConstants.blueLowerTrench.getY();

          Pose2d targetPose = new Pose2d(xCoord, yCoord, Rotation2d.fromDegrees(angle));

          PathConstraints constraints =
              new PathConstraints(
                  25.0, 14.0, Units.degreesToRadians(900), Units.degreesToRadians(720));

          return AutoBuilder.pathfindToPose(targetPose, constraints, 25.0);
        },
        Set.of(drive));
  }
}
