// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterStateStore;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.RebuiltField;

/** Add your docs here. */
public class AutoCommands {
    
    /**
     * A basic autonomous routine that drives to a specified y coordinate, aligns to the hub, and shoots for 15 seconds. We use a command like this instead of a preset path so that we can easily adjust our position on the field without having to regenerate paths
     * @param y the y coordinate to drive to. We use this instead of a preset command so that we can easily adjust our position on the field without having to regenerate paths
     * @param angle the angle to drive to. We use this instead of a preset command so that we can easily adjust our position on the field without having to regenerate paths
     */
    public static Command basicAuto(double y, double angle, Drive drive, ShooterStateStore shooterStateStore, ShooterSubsystem shooterSubsystem) {
        return Commands.sequence(
            PathCommands.goToCoordinate(() -> 2, () -> y, () -> angle),
            DriveCommands.autonomousAlignToCommand(
                drive, 
                () -> RebuiltField.getTranslationToHub2D().getAngle(), 
                () -> Constants.DriveConstants.kFastModeMultiplier),
            ShooterCommands.autoHub(shooterStateStore),
            Commands.waitSeconds(5),
            ShooterCommands.startFeeder(shooterSubsystem),
            Commands.waitSeconds(10),
            ShooterCommands.stopFeeder(shooterSubsystem)
        );
    }
}
