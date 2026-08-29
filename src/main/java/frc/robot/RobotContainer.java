// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCharacterizationCommands;
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
    
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private LoggedNetworkBoolean fieldOriented = new LoggedNetworkBoolean("Drive/FieldOriented", true);
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        drive   = Subsystems.createDrive(Constants.currentMode);
        vision  = Subsystems.createVision(Constants.currentMode, drive);
        shooter = Subsystems.createShooter(Constants.currentMode, shooterState);
        intake  = Subsystems.createIntake(Constants.currentMode);

        // Provide RebuiltField with live suppliers so AutoBuilder can flip paths and compute hub targeting.
        RebuiltField.setGlobalFlippedSupplier(new FlippedSupplier());
        RebuiltField.setGlobalRobotPoseSupplier(drive::getPose);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        configureAutoChooser();

        // Configure the button bindings
        configureDriverBindings();
        configureOperatorBindings();
        
        // schedule these commands to run when the robot is enabled
        CommandScheduler.getInstance().schedule(
        ShooterCommands.zeroHood(shooter)
        );
    }
    
    private void configureAutoChooser() {
        autoChooser.addOption("Left", AutoCommands.basicAuto(6, -45, drive, shooterState, shooter));
        autoChooser.addOption("Middle", AutoCommands.basicAuto(4, 0, drive, shooterState, shooter));
        autoChooser.addOption("Right", AutoCommands.basicAuto(2, 45, drive, shooterState, shooter));

        autoChooser.addOption("LeftIntakeAgitate", AutoCommands.basicAutoIntakeAgitate(6, -45, drive, shooterState, shooter, intake));
        autoChooser.addOption("MiddleIntakeAgitate", AutoCommands.basicAutoIntakeAgitate(4, 0, drive, shooterState, shooter, intake));
        autoChooser.addOption("RightIntakeAgitate", AutoCommands.basicAutoIntakeAgitate(2, 45, drive, shooterState, shooter, intake));

        autoChooser.addOption("LeftNeutral", AutoCommands.neutralAuto(6, -45, 90, drive, shooterState, shooter, intake));
        autoChooser.addOption("MiddleNeutral", AutoCommands.neutralAuto(4, 0, 90, drive, shooterState, shooter, intake));
        autoChooser.addOption("RightNeutral", AutoCommands.neutralAuto(2, 45, 90, drive, shooterState, shooter, intake));

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
            Commands.startEnd(() -> intake.startRoller(false), () -> intake.stopRoller(), intake)
        );

        driverController.leftBumper().whileTrue(
            Commands.startEnd(() -> intake.startRoller(true), () -> intake.stopRoller(), intake, shooter)
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
        operatorController.y().whileTrue(ShooterCommands.autoHub(shooterState));
        operatorController.b().whileTrue(ShooterCommands.manual(shooterState, operatorController::getLeftY, operatorController::getLeftX));

        operatorController.rightTrigger().whileTrue(ShooterCommands.shoot(shooter));
        operatorController.leftTrigger().whileTrue(ShooterCommands.reverseFeeder(shooter));

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
