// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.IntakeModule;
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
            Commands.race(
                 DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> 0,
                    () -> 0,
                    () -> RebuiltField.getTranslationToHub2D().getAngle(),
                    () -> 0.75),
                Commands.sequence(
                    Commands.runOnce(() -> {
                        shooterStateStore.set(RebuiltField.shooterStateForHub());
                    }),
                    Commands.waitSeconds(1.5),
                    ShooterCommands.startFeeder(shooterSubsystem),
                    Commands.waitSeconds(10),
                    ShooterCommands.stop(shooterSubsystem, shooterStateStore)
                )
            )
        );
    }
    public static Command basicAutoIntakeAgitate(double y, double angle, Drive drive, ShooterStateStore shooterStateStore, ShooterSubsystem shooterSubsystem, IntakeModule intake) {
        return Commands.sequence(
            PathCommands.goToCoordinate(() -> 2, () -> y, () -> angle),
            Commands.race(
                 DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> 0,
                    () -> 0,
                    () -> RebuiltField.getTranslationToHub2D().getAngle(),
                    () -> 0.75),
                Commands.sequence(
                    Commands.runOnce(() -> {
                        shooterStateStore.set(RebuiltField.shooterStateForHub());
                    }),
                    Commands.waitSeconds(1.5),
                    ShooterCommands.startFeeder(shooterSubsystem),
                    Commands.waitSeconds(10),
                    ShooterCommands.stop(shooterSubsystem, shooterStateStore)
                ),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    IntakeCommands.intakeAgitate(intake)
                )
            )
        );
    }public static Command neutralAuto(double y, double angle, double intakeAngle, Drive drive, ShooterStateStore shooterStateStore, ShooterSubsystem shooterSubsystem, IntakeModule intake) {
        return Commands.repeatingSequence(
                Commands.runOnce(
                    () -> {
                        intake.stopRoller();
                    }, intake
                ),
                PathCommands.goToCoordinate(() -> 2, () -> y, () -> angle),
                Commands.race(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> 0,
                        () -> 0,
                        () -> RebuiltField.getTranslationToHub2D().getAngle(),
                        () -> 0.75),
                    Commands.sequence(
                        Commands.runOnce(() -> {
                            shooterStateStore.set(RebuiltField.shooterStateForHub());
                        }),
                        Commands.waitSeconds(1.5),
                        ShooterCommands.startFeeder(shooterSubsystem),
                        Commands.waitSeconds(10),
                        ShooterCommands.stop(shooterSubsystem, shooterStateStore),
                        Commands.runOnce(() -> {
                            intake.lower();
                            intake.startRoller(false);
                        })
                    )
                ),
                PathCommands.goToCoordinate(() -> Constants.FieldConstants.center.getX(), () -> Constants.FieldConstants.center.getY(), () -> intakeAngle)
            );
    }
}
