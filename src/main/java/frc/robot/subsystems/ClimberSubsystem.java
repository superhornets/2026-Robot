// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;
  private final SparkClosedLoopController climberController;

  // Simulation
  private final SparkMaxSim climberMotorSim;
  private final SparkAbsoluteEncoderSim climberEncoderSim;
  private final DCMotor climberGearboxSim;
  private final SingleJointedArmSim climberSim;

  public ClimberSubsystem() {
    climberMotor = new SparkMax(Constants.Climber.CAN.kClimber, MotorType.kBrushless);

    SparkMaxConfig climberConfig = new SparkMaxConfig();
    climberConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .p(0.05)
        .i(0.0)
        .d(0.1)
        .positionWrappingEnabled(false);

    climberMotor.configure(
        climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    climberController = climberMotor.getClosedLoopController();

    // Simulation objects
    climberGearboxSim = DCMotor.getNEO(1);
    climberMotorSim = new SparkMaxSim(climberMotor, climberGearboxSim);
    climberEncoderSim = new SparkAbsoluteEncoderSim(climberMotor);

    climberSim =
        new SingleJointedArmSim(
            climberGearboxSim,
            Constants.Climber.SIM.kGearRatio,
            1.0, // mass kg (approx)
            1.0, // length meters (approx)
            Units.degreesToRadians(Constants.Climber.kMinAngle),
            Units.degreesToRadians(Constants.Climber.kMaxAngle),
            true,
            Units.degreesToRadians(Constants.Climber.kMinAngle));

    //  climberSim = new SingleJointedArmSim()
  }

  /** Begin moving the climber slowly toward its mechanical zero (applies small % output). */
  public void startZeroing() {
    climberMotor.set(-0.10); // conservative seek power, matches hood zeroing behavior
  }

  /** Stop zeroing motion (stop motor). */
  public void stopZeroing() {
    climberMotor.set(0.0);
  }

  /**
   * Returns true when the climber appears to be stalled against the mechanical stop.
   * Uses a heuristic: current above threshold AND near-zero encoder velocity.
   */
  public boolean isStalled() {
    double current = climberMotor.getOutputCurrent();
    double velocity = climberMotor.getEncoder().getVelocity(); // RPM
    // thresholds chosen to match the shooter hood zeroing pattern (conservative)
    return (current > 8.0) && (Math.abs(velocity) < 5.0);
  }

  /** Stop the motor and set the climber encoder position to zero. */
  public void setZero() {
    climberMotor.set(0.0);
    climberMotor.getEncoder().setPosition(0.0);
    // Reset controller setpoint to 0 to avoid unintended motion after zeroing
    climberController.setSetpoint(0.0, ControlType.kPosition, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
  }

  public void setPositionDegrees(double degrees) {
    double clamped =
        MathUtil.clamp(
            degrees, Constants.Climber.kMinAngle, Constants.Climber.kMaxAngle);
    double rotations = Units.degreesToRotations(clamped);
    climberController.setSetpoint(
        rotations, ControlType.kPosition, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
  }

  public void climberUp() {
    climberController.setSetpoint(Climber.kMaxAngle, ControlType.kPosition, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
  }
  public void climberDown() {
        climberController.setSetpoint(-Climber.kMaxAngle, ControlType.kPosition, com.revrobotics.spark.ClosedLoopSlot.kSlot0);

    // climberController.setSetpoint(-2, ControlType.kVelocity, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
    // lowering = 1;
  }
  public double getVelocity() {
    return climberMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Climber/Position")
  public double getPosition() {
    return climberMotor.getEncoder().getPosition();
  }

  public void climberStop() {
    climberController.setSetpoint(0, ControlType.kVelocity, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
  }
  
  public void simulationPeriodic() {
    // Apply motor voltage to the simulated arm
    climberSim.setInput(climberMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    climberSim.update(Constants.SIM.interval);

    climberMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(climberSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),
        Constants.SIM.interval);

    climberEncoderSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(climberSim.getVelocityRadPerSec())
            / Constants.Climber.SIM.kGearRatio,
        Constants.SIM.interval);
  }
}
