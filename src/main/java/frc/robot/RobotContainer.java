// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.AutoCommands;
import frc.robot.Commands.ClimberCommands;
import frc.robot.Commands.DriveCommands;
import frc.robot.Commands.IntakeCommands;
import frc.robot.Commands.PathCommands;
import frc.robot.Commands.ShooterCommands;
import frc.robot.Constants.Climber;
import frc.robot.Constants.Shooter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeModule;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.SpeedSupplier;
import frc.robot.subsystems.shooter.ShooterStateStore;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FlippedSupplier;
import frc.robot.util.RebuiltField;
import frc.robot.util.RebuiltMatch;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
    private final IntakeModule intake;
    // private final ClimberSubsystem climber;
    
    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    // Value Suppliers
    private final SpeedSupplier speedSupplier = new SpeedSupplier();
    private final ShooterStateStore shooterState = new ShooterStateStore();
    
    // Match Stuff
    private final RebuiltMatch rebuiltMatch = new RebuiltMatch();
    
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
    	shooter = new ShooterSubsystem(shooterState);
    	intake = new IntakeModule(
          Constants.Intake.CAN.kRightArm, Constants.Intake.CAN.kRightRoller, true, "Intake/Right");
        // climber = new ClimberSubsystem();
    	switch (Constants.currentMode) {
       case REAL:
         // Real robot, instantiate hardware IO implementations
         // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
         // a CANcoder
         drive =
             new Drive(
                 new GyroIOPigeon2(),
                 new ModuleIOTalonFX(TunerConstants.FrontLeft),
                 new ModuleIOTalonFX(TunerConstants.FrontRight),
                 new ModuleIOTalonFX(TunerConstants.BackLeft),
                 new ModuleIOTalonFX(TunerConstants.BackRight));
 
         vision =
             new Vision(
                 drive::addVisionMeasurement,
                 new VisionIOPhotonVision(camera0Name, robotToCamera0),
                 new VisionIOPhotonVision(camera1Name, robotToCamera1));
         break;
 
       case SIM:
         // Sim robot, instantiate physics sim IO implementations
         drive =
             new Drive(
                 new GyroIO() {},
                 new ModuleIOSim(TunerConstants.FrontLeft),
                 new ModuleIOSim(TunerConstants.FrontRight),
                 new ModuleIOSim(TunerConstants.BackLeft),
                 new ModuleIOSim(TunerConstants.BackRight));
 
         vision =
             new Vision(
                 drive::addVisionMeasurement,
                 new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                 new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
 
         break;
 
       default:
         // Replayed robot, disable IO implementations
         drive =
             new Drive(
                 new GyroIO() {},
                 new ModuleIO() {},
                 new ModuleIO() {},
                 new ModuleIO() {},
                 new ModuleIO() {});
 
         vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
         break;
     }
 
     // Provide RebuiltField with live suppliers so AutoBuilder can flip paths and compute hub targeting.
     RebuiltField.setGlobalFlippedSupplier(new FlippedSupplier());
     RebuiltField.setGlobalRobotPoseSupplier(drive::getPose);
     
        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        
        autoChooser.addOption("Left", AutoCommands.basicAuto(6, -45, drive, shooterState, shooter));
        autoChooser.addOption("Middle", AutoCommands.basicAuto(4, 0 , drive, shooterState, shooter));
        autoChooser.addOption("Right", AutoCommands.basicAuto(2, 45, drive, shooterState, shooter));

        // // Set up SysId routines
        // autoChooser.addOption(
        // "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        // autoChooser.addOption(
        // "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
        
        // Configure the button bindings
        configureButtonBindings();
        
        // schedule these commands to run when the robot is enabled
        CommandScheduler.getInstance().schedule(
        IntakeCommands.raise(intake),
        // ClimberCommands.zero(climber),
        ShooterCommands.zeroHood(shooter)
        );
    }
    
    /**
    * Use this method to define your button->command mappings. Buttons can be created by
    * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
    * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
        DriveCommands.joystickDrive(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        speedSupplier));
        
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
        
        driverController.leftTrigger().onTrue(Commands.runOnce(() -> { speedSupplier.setSlow(); }));
        driverController.leftTrigger().onFalse(Commands.runOnce(() -> { speedSupplier.reset(); }));
        driverController.rightTrigger().onTrue(Commands.runOnce(() -> { speedSupplier.setFast(); }));
        driverController.rightTrigger().onFalse(Commands.runOnce(() -> { speedSupplier.reset(); }));
        
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

        // driverController.rightBumper().onTrue(IntakeCommands.toggle(intake));
        driverController.povDown().onTrue(IntakeCommands.lower(intake));
        driverController.povUp().onTrue(IntakeCommands.raise(intake));

        driverController.rightBumper().onTrue(
            Commands.runOnce(
                () -> {
                    intake.startRoller(false);
                }, intake
            )
        );
        driverController.rightBumper().onFalse(
            Commands.runOnce(
                () -> { 
                    intake.stopRoller();
                 }, intake
            )
        );

        driverController.leftBumper().onTrue(
            Commands.runOnce(
                () -> {
                    intake.startRoller(true);
                }, intake, shooter
            )
        );
        driverController.leftBumper().onFalse(
            Commands.runOnce(
                () -> { 
                    intake.stopRoller();
                 }, intake, shooter
            )
        );

        driverController.start().onTrue(PathCommands.goToHubCommand());

        driverController
        .back()
        .whileTrue(
        PathCommands.ClosestTrench(
        drive,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> false));

        driverController.a().whileTrue(DriveCommands.shake(drive));
        
        operatorController.y().whileTrue(ShooterCommands.autoHub(shooterState));
        operatorController.b().whileTrue(ShooterCommands.manual(shooterState, operatorController::getLeftY, operatorController::getLeftX));
        

        operatorController.rightTrigger().whileTrue(ShooterCommands.shoot(shooter));
        operatorController.leftTrigger().whileTrue(ShooterCommands.reverseFeeder(shooter));
        
        // operatorController.povUp().onTrue(ClimberCommands.climberUp(climber));
        // operatorController.povDown().onTrue(ClimberCommands.climberDown(climber));
        
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
