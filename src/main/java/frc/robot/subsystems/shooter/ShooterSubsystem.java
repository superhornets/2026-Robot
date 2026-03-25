// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterSubsystem extends SubsystemBase {

  ShooterStateStore stateStore;;
  // HARDWARE OBJECTS

  // flywheel
  private SparkFlex flywheelMotorLeft;
  private SparkClosedLoopController flywheelControllerLeft;
  private SparkFlex flywheelMotorRight;
  private SparkClosedLoopController flywheelControllerRight;

  // other
  private SparkMax hoodMotor;
  private SparkClosedLoopController hoodController;
  private SparkFlex agitatorMotor;
  private SparkClosedLoopController agitatorController;
  private SparkFlex feederMotor;
  private SparkClosedLoopController feederController;

  private LoggedNetworkNumber flywheelP = new LoggedNetworkNumber("Shooter/FlywheelP", 0.0001);
  private LoggedNetworkNumber flywheelD = new LoggedNetworkNumber("Shooter/FlywheelD", 0.0002);

  // SIMULATION OBJECTS
  private SparkMaxSim hoodMotorSim;
  private DCMotor hoodGearboxSim;
  private ElevatorSim hoodSim;
  private SparkRelativeEncoderSim hoodEncoderSim;
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
  public ShooterSubsystem(ShooterStateStore store) {
    this.stateStore = store;

    // Setup Motors and Controllers
    hoodMotor = new SparkMax(Constants.Shooter.CAN.kHood, MotorType.kBrushless);

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(.1)
        .i(0)
        .d(0.01)
        .outputRange(-1, 1)
        .allowedClosedLoopError(Units.degreesToRotations(0.2), ClosedLoopSlot.kSlot0);

    hoodMotor.configure(
        hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    hoodController = hoodMotor.getClosedLoopController();

    configureFlywheelMotors();

    feederMotor = new SparkFlex(Constants.Shooter.CAN.kFeeder, MotorType.kBrushless);
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .closedLoop
        .p(0.0006)
        .i(0)
        .d(0.001)
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
        .p(0.0005)
        .i(0)
        .d(0.001)
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    agitatorMotor.configure(
        agigitatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    agitatorController = agitatorMotor.getClosedLoopController();

    // SIMULATION OBJECTS
    hoodGearboxSim = DCMotor.getNeo550(1);
    hoodMotorSim = new SparkMaxSim(hoodMotor, hoodGearboxSim);
    hoodEncoderSim = hoodMotorSim.getRelativeEncoderSim();
    hoodSim = new ElevatorSim(hoodGearboxSim, 9, .01, Units.inchesToMeters(1.5), 0, Units.inchesToMeters(6), true, 0);


    // Flywheel sim
    flywheelGearboxSim = DCMotor.getNeoVortex(2);
    flywheelMotorSim = new SparkFlexSim(flywheelMotorRight, flywheelGearboxSim);
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
                Constants.Shooter.SIM.kFlywheelMOI * 10,
                Constants.Shooter.SIM.kFlywheelGearRatio),
            agitatorGearboxSim);

    hoodMotor.getEncoder().setPosition(0);
  }

  public void periodic() {
    setState(stateStore.get());
  }

  public void setState(ShooterState state) {
    setHoodAngle(state.angle);
    startFlywheel(state.speed);
  }

  public void setHoodAngle(double angleRotations) {
    double value = Math.max(Constants.Shooter.kHoodMinAngle, Math.min(Constants.Shooter.kHoodMaxAngle, angleRotations));
    hoodController.setSetpoint(value, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private void configureFlywheelMotors() {
  SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    flywheelConfig
        .smartCurrentLimit(20, 40, 1000)
        .closedLoop
        .p(flywheelP.get())
        .i(0)
        .d(flywheelD.get())
        .maxMotion
        .maxAcceleration(10_000, ClosedLoopSlot.kSlot0);
    flywheelConfig.encoder
      .positionConversionFactor(1.0)
      .velocityConversionFactor(1.0);


    flywheelMotorRight = new SparkFlex(Constants.Shooter.CAN.kFlywheelRight, MotorType.kBrushless);
    flywheelMotorRight.configure(
        flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelControllerRight = flywheelMotorRight.getClosedLoopController();

    flywheelMotorLeft = new SparkFlex(Constants.Shooter.CAN.kFlywheelLeft, MotorType.kBrushless);
    flywheelMotorLeft.configure(
        flywheelConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelControllerLeft = flywheelMotorLeft.getClosedLoopController();
  }

  public void startFlywheel(double speedRPM) {
    double speedClamped =
        MathUtil.clamp(
            speedRPM, Constants.Shooter.kFlywheelMinSpeed, Constants.Shooter.kFlywheelMaxSpeed);
    flywheelControllerLeft.setSetpoint(
        speedClamped, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    flywheelControllerRight.setSetpoint(
        speedClamped, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  public void stopFlywheel(boolean coast) {
    flywheelControllerRight.setSetpoint(
        0.0,
        coast ? ControlType.kDutyCycle : ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0);
    flywheelControllerLeft.setSetpoint(
        0.0,
        coast ? ControlType.kDutyCycle : ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0);
  }

  public void startFeeder() {
      feederController.setSetpoint(
        3000, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
      agitatorController.setSetpoint(
        1500, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  public void startReverseFeeder() {
    feederController.setSetpoint(-200, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    agitatorController.setSetpoint(1000, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    startFlywheel(-200);
  }

  public void stopReverseFeeder() {
    stopFeeder();
    stopFlywheel(false);
  }

  public void stopFeeder() {
       feederController.setSetpoint(0.0, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
       agitatorController.setSetpoint(0.0, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
   }

  // Hood zeroing utilities (patterned after ClimberSubsystem zeroing)
  /** Begin moving the hood slowly toward its mechanical zero (applies small % output). */
  public void startHoodZeroing() {
    // apply a small negative percent to seek the hard stop; tuned low to avoid damage
    hoodMotor.set(-0.10);
  }

  /** Stop zeroing motion (stop motor). */
  public void stopHoodZeroing() {
    hoodMotor.set(0.0);
  }

  /**
   * Returns true when the hood appears to be stalled against the mechanical stop.
   * Uses a simple heuristic: current above threshold AND near-zero encoder velocity.
   */
  public boolean isHoodStalled() {
    double current = hoodMotor.getOutputCurrent();
    double velocity = hoodMotor.getEncoder().getVelocity(); // RPM
    // threshold values chosen conservatively; adjust if needed for your hardware
    return (current > 8.0) && (Math.abs(velocity) < 5.0);
  }

  /** Stop the motor and set the hood encoder position to zero. */
  public void setHoodZero() {
    hoodMotor.set(0.0);
    // Reset encoder position (rotations) to zero at the mechanical stop
    hoodMotor.getEncoder().setPosition(0.0);
    // Also reset controller setpoint to 0 to avoid unwanted motion after zeroing
    hoodController.setSetpoint(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @AutoLogOutput(key = "Shooter/hoodAngle")
  public double getHoodAngle() {
    return hoodMotor.getEncoder().getPosition();
  }

    @AutoLogOutput(key = "Shooter/hoodAngleSetpoint")
  public double getHoodAngleSet() {
    return hoodController.getSetpoint();
  }

  @AutoLogOutput(key = "Shooter/FlywheelVelocitySetpointRPM")
  public double getFlywheelVelocitySetpointRPM() {
    return flywheelControllerRight.getSetpoint();
  }

  @AutoLogOutput(key = "Shooter/FlywheelVelocityRPM")
  public double getFlywheelVelocityRPM() {
    return flywheelMotorRight.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Shooter/isAtSpeed")
  public boolean getIsAtSpeed() {
    return flywheelControllerRight.isAtSetpoint();
  }

  @AutoLogOutput(key = "Shooter/Ready")
  public boolean getReady() {
    return getIsAtSpeed() && hoodController.isAtSetpoint();
  }

  public void simulationPeriodic() {
    // Hood
    hoodSim.setInput(hoodMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
    hoodSim.update(Constants.SIM.interval);
    hoodMotorSim.iterate(hoodSim.getVelocityMetersPerSecond(), hoodSim.getPositionMeters(), Constants.SIM.interval);

    // Flywheel
    flywheelMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            flywheelSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds
        flywheelSim.setInput(2 * flywheelMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    flywheelSim.update(Constants.SIM.interval);

    // Feeder
    feederMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            feederSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds
    feederSim.setInput(feederMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    feederSim.update(Constants.SIM.interval);

    // Agitator
    agitatorMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            agitatorSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        Constants.SIM.interval); // Time interval, in Seconds
    agitatorSim.setInput(agitatorMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    agitatorSim.update(Constants.SIM.interval);
  }
}
