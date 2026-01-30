// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.units.Units;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Vision;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Commands.OrchestraPlayCommand;
import frc.robot.Commands.OrchestraStopCommand;
import frc.robot.Commands.SquareAutoCommand;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.VisionSystem;

public class RobotContainer {
private final SendableChooser<Command> autoChooser;
    private double SpeedMultiplier = 0.25;
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * SpeedMultiplier; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * SpeedMultiplier; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    TalonFX[] instruments = {
        new TalonFX(11), 
        new TalonFX(12), 
        new TalonFX(21), 
        new TalonFX(22), 
        new TalonFX(31), 
        new TalonFX(32), 
        new TalonFX(41), 
        new TalonFX(42)};
    public final Orchestra m_orchestra = new Orchestra();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final VisionSystem vision = new VisionSystem(drivetrain::addVisionMeasurement);

    public RobotContainer() {
        configureBindings();
        
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);


        // Auto sandbox
        // Square Auto
        Command squareAuto = new PathPlannerAuto("Square");
        joystick.x().onTrue(squareAuto); // Not sure if this will work. Might need a custom squareAutoCommand object that calls Command.schedule() for the execute function.
        // joystick.x().onTrue(new SquareAutoCommand(squareAuto));

        try
        {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Right");
            
            // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
            PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), 
                Units.degreesToRadians(720));

                
            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                    path,
                    constraints);

            joystick.y().onTrue(pathfindingCommand); // Not sure if this will work. Might need a custom squareAutoCommand object that calls Command.schedule() for the execute function.

        }
        catch(Exception e)
        {
            DriverStation.reportError(e.getMessage(),false);
        }
        

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0, 
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        // // Orchestra
        for(TalonFX talon : instruments)
        {
            m_orchestra.addInstrument(talon);
        }
        m_orchestra.loadMusic("thunder.chrp");
        joystick.povUp().onTrue(new OrchestraPlayCommand(m_orchestra));
        joystick.povDown().onTrue(new OrchestraStopCommand(m_orchestra));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
