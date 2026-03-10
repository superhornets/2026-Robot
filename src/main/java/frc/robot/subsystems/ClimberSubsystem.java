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
import org.littletonrobotics.junction.AutoLogOutput;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;
  private final SparkClosedLoopController climberController;

  // Simulation
  private final SparkMaxSim climberMotorSim;
  // private final SparkAbsoluteEncoderSim climberEncoderSim;
  private final SparkAbsoluteEncoderSim climberEncoderSim;
  private final DCMotor climberGearboxSim;
  private final SingleJointedArmSim climberSim;

  public ClimberSubsystem() {
    climberMotor = new SparkMax(Constants.Climber.CAN.kClimber, MotorType.kBrushless);

    SparkMaxConfig climberConfig = new SparkMaxConfig();
    climberConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop
        //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(1.0)
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
            Units.degreesToRadians(Constants.Climber.kMinAngleDegrees),
            Units.degreesToRadians(Constants.Climber.kMaxAngleDegrees),
            true,
            Units.degreesToRadians(Constants.Climber.kMinAngleDegrees));

    //  climberSim = new SingleJointedArmSim()
  }

  @Override
  public void periodic() {
    // Nothing required for now.
  }

  public void setPositionDegrees(double degrees) {
    double clamped =
        MathUtil.clamp(
            degrees, Constants.Climber.kMinAngleDegrees, Constants.Climber.kMaxAngleDegrees);
    double rotations = Units.degreesToRotations(clamped);
    climberController.setSetpoint(
        rotations, ControlType.kPosition, com.revrobotics.spark.ClosedLoopSlot.kSlot0);
  }

  @AutoLogOutput(key = "Climber/angleDegrees")
  public double getAngleDegrees() {
    // Encoder position reported in rotations -> convert to degrees
    double rotations = climberMotor.getAbsoluteEncoder().getPosition();
    return Units.rotationsToDegrees(rotations);
  }

  public void simulationPeriodic() {
    getAngleDegrees();
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
