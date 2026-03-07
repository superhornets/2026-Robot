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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathCommands {
  /** Creates a new PathCommands. */
  public PathCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static Command goToHubCommand() {

    Pose2d targetPose = new Pose2d(3, 4, Rotation2d.fromDegrees(0));

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(17.0, 8.0, Units.degreesToRadians(900), Units.degreesToRadians(720));

    Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose, constraints, 0.0);
    return pathfindingCommand;
  }
}
