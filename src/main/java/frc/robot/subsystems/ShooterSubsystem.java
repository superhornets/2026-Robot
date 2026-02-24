// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  // HARDWARE OBJECTS
  private SparkMax hoodMotor;
  private SparkClosedLoopController hoodController;
  private SparkFlex flywheelMotor;
  private SparkClosedLoopController flywheelController;
  private SparkFlex agitatorMotor;
  private SparkClosedLoopController agitatorController;
  private SparkFlex feederMotor;
  private SparkClosedLoopController feederController;

  // SIMULATION OBJECTS
  private SparkMaxSim hoodMotorSim;
  private SparkAbsoluteEncoderSim hoodEncoderSim;
  private DCMotor hoodGearboxSim;
  private SingleJointedArmSim hoodSim;
  private SparkFlexSim flywheelMotorSim;
  private FlywheelSim flywheelSim;
  private DCMotor flywheelGearboxSim;
  // Feeder simulation (matches flywheel)
  private SparkFlexSim feederMotorSim;
  private FlywheelSim feederSim;
  private DCMotor feederGearboxSim;
  // Agitator simulation (matches flywheel)
  private SparkFlexSim agitatorMotorSim;
  private FlywheelSim agitatorSim;
  private DCMotor agitatorGearboxSim;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // Setup Motors and Controllers
    hoodMotor = new SparkMax(Constants.Shooter.CAN.kHood, MotorType.kBrushless);

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(10)
        .i(0)
        .d(1)
        .positionWrappingEnabled(true)
        .allowedClosedLoopError(Units.degreesToRotations(0.2), ClosedLoopSlot.kSlot0)
        .maxMotion
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedProfileError(Units.degreesToRotations(0.2))
        .cruiseVelocity(120)
        .maxAcceleration(6_000.0, ClosedLoopSlot.kSlot0);

    hoodMotor.configure(
        hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    hoodController = hoodMotor.getClosedLoopController();

    flywheelMotor = new SparkFlex(Constants.Shooter.CAN.kFlywheel, MotorType.kBrushless);
    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    flywheelConfig
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .p(0.1)
        .i(0)
        .d(0.1)
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    flywheelMotor.configure(
        flywheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    flywheelController = flywheelMotor.getClosedLoopController();

    feederMotor = new SparkFlex(Constants.Shooter.CAN.kFeeder, MotorType.kBrushless);
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .closedLoop
        .p(0.1)
        .i(0)
        .d(0.1)
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    feederMotor.configure(
        feederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    feederController = feederMotor.getClosedLoopController();

    agitatorMotor = new SparkFlex(Constants.Shooter.CAN.kAgitator, MotorType.kBrushless);
    SparkFlexConfig agigitatorConfig = new SparkFlexConfig();
    agigitatorConfig
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .p(0.1)
        .i(0)
        .d(0.1)
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    agitatorMotor.configure(
        agigitatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    agitatorController = agitatorMotor.getClosedLoopController();

    // SIMULATION OBJECTS
    hoodGearboxSim = DCMotor.getNEO(1);
    hoodMotorSim = new SparkMaxSim(hoodMotor, hoodGearboxSim);
    hoodEncoderSim = new SparkAbsoluteEncoderSim(hoodMotor);

    hoodSim =
        new SingleJointedArmSim(
            hoodGearboxSim,
            Constants.Shooter.SIM.kHoodGearRatio,
            1,
            1,
            Units.degreesToRadians(Constants.Shooter.kHoodMaxAngleDegrees),
            Units.degreesToRadians(Constants.Shooter.kHoodMinAngleDegrees),
            true,
            Units.degreesToRadians(Constants.Shooter.kHoodMaxAngleDegrees));

    // Flywheel sim
    flywheelGearboxSim = DCMotor.getNeoVortex(1);
    flywheelMotorSim = new SparkFlexSim(flywheelMotor, flywheelGearboxSim);

    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                flywheelGearboxSim,
                Constants.Shooter.SIM.kFlywheelMOI,
                Constants.Shooter.SIM.kFlywheelGearRatio),
            flywheelGearboxSim);

    // Feeder sim
    feederGearboxSim = DCMotor.getNeoVortex(1);
    feederMotorSim = new SparkFlexSim(feederMotor, feederGearboxSim);
    feederSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                feederGearboxSim,
                Constants.Shooter.SIM.kFlywheelMOI,
                Constants.Shooter.SIM.kFlywheelGearRatio),
            feederGearboxSim);

    // Agitator sim
    agitatorGearboxSim = DCMotor.getNeoVortex(1);
    agitatorMotorSim = new SparkFlexSim(agitatorMotor, agitatorGearboxSim);
    agitatorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                agitatorGearboxSim,
                Constants.Shooter.SIM.kFlywheelMOI,
                Constants.Shooter.SIM.kFlywheelGearRatio),
            agitatorGearboxSim);
  }

  public void periodic() {
    // Nothing to do here, as the subsystem is fully closed-loop in simulation and on the real
    // robot. The only thing we need to do is update the simulation objects in simulationPeriodic().
  }

  /** Lowers the arm and starts the roller at the intake speed. */
  public void setHoodAngle(double angleDegrees) {
    double angleInRotations =
        Units.degreesToRotations(
            MathUtil.clamp(
                angleDegrees,
                Constants.Shooter.kHoodMinAngleDegrees,
                Constants.Shooter.kHoodMaxAngleDegrees));
    hoodController.setSetpoint(
        angleInRotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  public void startFlywheel(double speedRPM) {
    double speedClamped =
        MathUtil.clamp(
            speedRPM, Constants.Shooter.kFlywheelMinSpeed, Constants.Shooter.kFlywheelMaxSpeed);
    flywheelController.setSetpoint(
        speedClamped, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  public void stopFlywheel() {
    flywheelController.setSetpoint(0.0, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  public void startFeeder() {
    feederController.setSetpoint(
        1500, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    agitatorController.setSetpoint(
        500, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  public void reverseFeeder() {
    feederController.setSetpoint(
        -200, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    agitatorController.setSetpoint(
        1000, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  public void stopFeeder() {
    feederController.setSetpoint(0.0, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    agitatorController.setSetpoint(0.0, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  public double getAngleRadians() {
    return hoodMotor.getAbsoluteEncoder().getPosition();
  }

  public double getRollerVelocityRPM() {
    return flywheelMotor.getEncoder().getVelocity();
  }

  public void simulationPeriodic() {
    // Hood
    hoodSim.setInput(hoodMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    hoodSim.update(Constants.SIM.interval);
    hoodMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            hoodSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),
        Constants.SIM.interval);

    hoodEncoderSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(hoodSim.getVelocityRadPerSec())
            / Constants.Shooter.SIM.kHoodGearRatio,
        Constants.SIM.interval);

    // Flywheel
    flywheelSim.setInput(flywheelMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    flywheelSim.update(Constants.SIM.interval);
    flywheelMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            flywheelSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds

    // Feeder
    feederSim.setInput(feederMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    feederSim.update(Constants.SIM.interval);
    feederMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            feederSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds

    // Agitator
    agitatorSim.setInput(agitatorMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    agitatorSim.update(Constants.SIM.interval);
    agitatorMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            agitatorSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds
  }
}
