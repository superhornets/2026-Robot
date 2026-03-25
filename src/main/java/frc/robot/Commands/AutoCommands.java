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
    public static Command basicAuto(Drive drive, ShooterStateStore shooterStateStore, ShooterSubsystem shooterSubsystem) {
        return Commands.sequence(
            PathCommands.goToHubCommand(),
            Commands.waitSeconds(3),
            DriveCommands.autonomousAlignToCommand(
                drive, 
                () -> RebuiltField.getTranslationToHub2D().getAngle(), 
                () -> Constants.DriveConstants.kFastModeMultiplier),
            ShooterCommands.autoHub(shooterStateStore),
            ShooterCommands.waitForReady(shooterSubsystem),
            Commands.parallel(
                ShooterCommands.shoot(shooterSubsystem),
                DriveCommands.shake(drive)
            ),
            Commands.waitSeconds(15)
        );
    }
}
