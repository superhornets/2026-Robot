// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathCommands {
  /** Creates a new PathCommands. */
  public PathCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static Command goToHubCommand(BooleanSupplier isFlipped) {

    Pose2d targetPose =
        isFlipped.getAsBoolean()
            ? new Pose2d(14, 4, Rotation2d.fromDegrees(180))
            : new Pose2d(3, 4, Rotation2d.fromDegrees(0));

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0); // Goal end velocity in meters/sec);// Rotation delay distance in meters. This is
    // how far the robot should travel before attempting to rotate.
    return pathfindingCommand;
  }
}
