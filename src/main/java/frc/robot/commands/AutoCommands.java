// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterStateStore;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.RebuiltField;

public class AutoCommands {

    private static Command buildShootRace(Drive drive, ShooterStateStore store, ShooterSubsystem shooter, double shootSecs, Command... extraRaceCmds) {
        Command alignCmd = DriveCommands.joystickDriveAtAngle(
            drive, () -> 0, () -> 0,
            () -> RebuiltField.getTranslationToHub2D().getAngle(), () -> 0.75);
        Command shootSeq = Commands.sequence(
            Commands.runOnce(() -> store.set(RebuiltField.shooterStateForHub())),
            Commands.waitSeconds(1.5),
            ShooterCommands.triggerFeed(shooter),
            Commands.waitSeconds(shootSecs),
            ShooterCommands.idle(shooter, store)
        );
        Command[] allCmds = new Command[2 + extraRaceCmds.length];
        allCmds[0] = alignCmd;
        allCmds[1] = shootSeq;
        System.arraycopy(extraRaceCmds, 0, allCmds, 2, extraRaceCmds.length);
        return Commands.race(allCmds);
    }

    /**
     * A basic autonomous routine that drives to a specified y coordinate, aligns to the hub, and shoots.
     * Uses a command-based approach so position can be adjusted without regenerating paths.
     */
    public static Command basicAuto(double y, double angle, Drive drive, ShooterStateStore shooterStateStore, ShooterSubsystem shooterSubsystem) {
        return Commands.sequence(
            PathCommands.goToCoordinate(() -> 2, () -> y, () -> angle, drive),
            buildShootRace(drive, shooterStateStore, shooterSubsystem, 10)
        );
    }

    public static Command basicAutoIntakeAgitate(double y, double angle, Drive drive, ShooterStateStore shooterStateStore, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake) {
        return Commands.sequence(
            PathCommands.goToCoordinate(() -> 2, () -> y, () -> angle, drive),
            buildShootRace(drive, shooterStateStore, shooterSubsystem, 10,
                Commands.sequence(Commands.waitSeconds(1.5), IntakeCommands.intakeAgitate(intake)))
        );
    }

    public static Command neutralAuto(double y, double angle, double intakeAngle, Drive drive, ShooterStateStore shooterStateStore, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake) {
        return Commands.sequence(
            Commands.runOnce(() -> intake.stopRoller(), intake),
            PathCommands.goToCoordinate(() -> 2, () -> y, () -> angle, drive),
            Commands.race(
                DriveCommands.joystickDriveAtAngle(
                    drive, () -> 0, () -> 0,
                    () -> RebuiltField.getTranslationToHub2D().getAngle(), () -> 0.75),
                Commands.sequence(
                    Commands.runOnce(() -> shooterStateStore.set(RebuiltField.shooterStateForHub())),
                    Commands.waitSeconds(1.5),
                    ShooterCommands.triggerFeed(shooterSubsystem),
                    Commands.waitSeconds(7),
                    ShooterCommands.idle(shooterSubsystem, shooterStateStore),
                    Commands.runOnce(() -> { intake.lower(); intake.startRoller(false); })
                )
            ),
            PathCommands.goToCoordinate(() -> Constants.FieldConstants.center.getX(), () -> y, () -> intakeAngle, drive)
        );
    }
}
