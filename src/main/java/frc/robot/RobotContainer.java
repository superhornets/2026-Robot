// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.PathCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SpeedCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SpeedSupplier;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterStateStore;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FlippedSupplier;
import frc.robot.util.RebuiltField;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    
    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    // Value Suppliers
    private final SpeedSupplier speedSupplier = new SpeedSupplier();
    private final ShooterStateStore shooterState = new ShooterStateStore();

    // Triggers
    private final Trigger shooterReady;
    private final Trigger alignedToHub;
    
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private LoggedNetworkBoolean fieldOriented = new LoggedNetworkBoolean("Drive/FieldOriented", true);
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        drive   = Subsystems.createDrive(Constants.currentMode);
        vision  = Subsystems.createVision(Constants.currentMode, drive);
        shooter = Subsystems.createShooter(Constants.currentMode, shooterState);
        intake  = Subsystems.createIntake(Constants.currentMode);
        
        shooterReady = new Trigger(shooter::getReady).debounce(0.05);
        alignedToHub = new Trigger(drive::isAlignedToHub).debounce(0.05);

        // Provide RebuiltField with live suppliers so AutoBuilder can flip paths and compute hub targeting.
        RebuiltField.setGlobalFlippedSupplier(new FlippedSupplier());
        RebuiltField.setGlobalRobotPoseSupplier(drive::getPose);

        setupCommandsAndEvents();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        configureAutoChooser();

        // Configure the button bindings
        configureDriverBindings();
        configureOperatorBindings();
        
        // schedule these commands to run when the robot is enabled
        CommandScheduler.getInstance().schedule(
        ShooterCommands.homeHood(shooter)
        );
    }
    
    private void setupCommandsAndEvents() {
        // Register named commands for PathPlanner autos
        NamedCommands.registerCommand("shooter_idle", ShooterCommands.idle(shooter, shooterState));
        NamedCommands.registerCommand("shooter_primeShooter", ShooterCommands.primeShooter(shooterState));
        NamedCommands.registerCommand("shoot", ShooterCommands.feed(shooter));
        NamedCommands.registerCommand("intake_lower", IntakeCommands.lower(intake));
        NamedCommands.registerCommand("intake", IntakeCommands.intake(false, intake));
        NamedCommands.registerCommand("intake_reverse", IntakeCommands.intake(true, intake));
        NamedCommands.registerCommand("intake_agitate", IntakeCommands.intakeAgitate(intake));

        // Register event commands for PathPlanner autos
        new EventTrigger("event_auto_intake").whileTrue(IntakeCommands.intake(false, intake));
        new EventTrigger("event_auto_shoot").whileTrue(ShooterCommands.autoShoot(shooter, shooterState, alignedToHub));
    }

    private void configureAutoChooser() {
        autoChooser.addOption("Left", AutoCommands.basicAuto(6, -45, drive, shooterState, shooter));
        autoChooser.addOption("Middle", AutoCommands.basicAuto(4, 0, drive, shooterState, shooter));
        autoChooser.addOption("Right", AutoCommands.basicAuto(2, 45, drive, shooterState, shooter));
        autoChooser.addOption("RightShootIntakeShoot", AutoBuilder.buildAuto("RightShootIntakeShoot"));
        autoChooser.addOption("LeftShootIntakeShoot", new PathPlannerAuto("RightShootIntakeShoot", true));

        // autoChooser.addOption("LeftIntakeAgitate", AutoCommands.basicAutoIntakeAgitate(6, -45, drive, shooterState, shooter, intake));
        // autoChooser.addOption("MiddleIntakeAgitate", AutoCommands.basicAutoIntakeAgitate(4, 0, drive, shooterState, shooter, intake));
        // autoChooser.addOption("RightIntakeAgitate", AutoCommands.basicAutoIntakeAgitate(2, 45, drive, shooterState, shooter, intake));

        // autoChooser.addOption("LeftNeutral", AutoCommands.neutralAuto(6, -45, 90, drive, shooterState, shooter, intake));
        // autoChooser.addOption("MiddleNeutral", AutoCommands.neutralAuto(4, 0, 90, drive, shooterState, shooter, intake));
        // autoChooser.addOption("RightNeutral", AutoCommands.neutralAuto(2, 45, 90, drive, shooterState, shooter, intake));

        // // Set up SysId routines
        // autoChooser.addOption(
        // "Drive Wheel Radius Characterization", DriveCharacterizationCommands.wheelRadiusCharacterization(drive));
        // autoChooser.addOption(
        // "Drive Simple FF Characterization", DriveCharacterizationCommands.feedforwardCharacterization(drive));
        // autoChooser.addOption(
        // "Drive SysId (Quasistatic Forward)",
        // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        // "Drive SysId (Quasistatic Reverse)",
        // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        // "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        // "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureDriverBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
        DriveCommands.joystickDrive(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        speedSupplier,
        fieldOriented));
        
        // Lock to Hub when Y button is held
        driverController
        .y()
        .whileTrue(
        DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> RebuiltField.getTranslationToHub2D().getAngle(),
        speedSupplier));
        
        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        
        driverController.leftTrigger().whileTrue(SpeedCommands.slowCommand(speedSupplier));
        driverController.rightTrigger().whileTrue(SpeedCommands.fastCommand(speedSupplier));
        
        // Reset gyro to 0° when B button is pressed
        driverController
        .b()
        .onTrue(
        Commands.runOnce(
        () ->
        drive.setPose(
        new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
        drive)
        .ignoringDisable(true));

        driverController.povDown().onTrue(IntakeCommands.lower(intake));
        driverController.povUp().onTrue(IntakeCommands.raise(intake));

        driverController.rightBumper().whileTrue(
            IntakeCommands.intake(false, intake)
        );

        driverController.leftBumper().whileTrue(
            IntakeCommands.intake(true, intake)
        );

        driverController.start().onTrue(PathCommands.goToHubCommand(drive));

        driverController
        .back()
        .whileTrue(
        PathCommands.ClosestTrench(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> false));

        driverController.a().whileTrue(DriveCommands.shake(drive));
    }

    private void configureOperatorBindings() {
        operatorController.y().whileTrue(ShooterCommands.primeShooter(shooterState));
        operatorController.b().whileTrue(ShooterCommands.adjustAim(shooterState, operatorController::getLeftY, operatorController::getLeftX));

        operatorController.rightTrigger().whileTrue(ShooterCommands.feed(shooter));
        operatorController.leftTrigger().whileTrue(ShooterCommands.unjam(shooter));

        operatorController.a().whileTrue(IntakeCommands.intakeAgitate(intake));
    }
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
    
    public Pose2d getPose() {
        return drive.getPose();
    }
}
