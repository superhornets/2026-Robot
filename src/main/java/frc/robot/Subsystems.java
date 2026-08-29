// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterStateStore;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/** Static factory methods for creating each subsystem wired to the correct IO layer. */
public final class Subsystems {
    private Subsystems() {}

    public static Drive createDrive(Constants.Mode mode) {
        switch (mode) {
            case REAL:
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn,
                // and a CANcoder
                return new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));
            case SIM:
                return new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontRight),
                    new ModuleIOSim(TunerConstants.BackLeft),
                    new ModuleIOSim(TunerConstants.BackRight));
            default:
                return new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
        }
    }

    public static Vision createVision(Constants.Mode mode, Drive drive) {
        switch (mode) {
            case REAL:
                return new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVision(camera0Name, robotToCamera0),
                    new VisionIOPhotonVision(camera1Name, robotToCamera1));
            case SIM:
                return new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                    new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
            default:
                return new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        }
    }

    public static ShooterSubsystem createShooter(Constants.Mode mode, ShooterStateStore shooterState) {
        switch (mode) {
            case REAL:
                return new ShooterSubsystem(new ShooterIOSparkMax(), shooterState);
            case SIM:
                return new ShooterSubsystem(new ShooterIOSim(), shooterState);
            default:
                return new ShooterSubsystem(new ShooterIO() {}, shooterState);
        }
    }

    public static IntakeSubsystem createIntake(Constants.Mode mode) {
        switch (mode) {
            case REAL:
                return new IntakeSubsystem(
                    new IntakeIOHardware(Constants.Intake.CAN.kRightArm, Constants.Intake.CAN.kRightRoller, true),
                    "Intake/Right", IntakeConstants.kIntakeRollerSpeedRight);
            case SIM:
                return new IntakeSubsystem(
                    new IntakeIOSim(), "Intake/Right", IntakeConstants.kIntakeRollerSpeedRight);
            default:
                return new IntakeSubsystem(
                    new IntakeIO() {}, "Intake/Right", IntakeConstants.kIntakeRollerSpeedRight);
        }
    }
}
