// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  // Converts ElevatorSim linear position (meters) → motor shaft rotations:
  //   rotations = meters × gearRatio / (2π × drumRadius)
  // Drive gear is 1" diameter → 0.5" radius; 9:1 gearbox.
  private static final double HOOD_METERS_TO_ROTATIONS =
      ShooterConstants.SIM.kHoodGearRatio / (2 * Math.PI * Units.inchesToMeters(0.5));

  // Hood: Neo 550 → 9:1 gearbox → rack & pinion with 1" diameter drive gear
  private final DCMotor hoodGearbox = DCMotor.getNeo550(1);
  private final ElevatorSim hoodSim = new ElevatorSim(
      hoodGearbox,
      ShooterConstants.SIM.kHoodGearRatio,
      0.01,                       // carriage mass kg
      Units.inchesToMeters(0.5),  // 1" diameter drive gear → 0.5" radius
      0,
      Units.inchesToMeters(6),
      true,
      0);
  private double hoodAppliedOutput = 0.0;
  private double hoodPositionOffset = 0.0;
  private double hoodSetpoint = 0.0;

  // Flywheel
  private final DCMotor flywheelGearbox = DCMotor.getNeoVortex(2);
  private final FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              flywheelGearbox,
              ShooterConstants.SIM.kFlywheelMOI,
              ShooterConstants.SIM.kFlywheelGearRatio),
          flywheelGearbox);
  private double flywheelAppliedVolts = 0.0;
  private double flywheelSetpointRPM = 0.0;

  // Spindexer: NeoVortex (SparkFlex), 4:1 gear ratio, 2 in²lb MOI
  private final DCMotor spindexerGearbox = DCMotor.getNeoVortex(1);
  private final FlywheelSim spindexerSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          spindexerGearbox,
          ShooterConstants.SIM.kSpindexerMOI,
          ShooterConstants.SIM.kSpindexerGearRatio),
      spindexerGearbox);
  private double spindexerAppliedVolts = 0.0;

  // Feeder: Neo 1.0, 4:1 gear ratio
  private final DCMotor feederGearbox = DCMotor.getNEO(1);
  private final FlywheelSim feederSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          feederGearbox,
          ShooterConstants.SIM.kFeederMOI,
          ShooterConstants.SIM.kFeederGearRatio),
      feederGearbox);
  private double feederAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double dt = Constants.SIM.interval;
    double batteryVolts = RoboRioSim.getVInVoltage();

    // Hood
    hoodSim.setInput(hoodAppliedOutput * batteryVolts);
    hoodSim.update(dt);
    inputs.hoodPositionRotations =
        (hoodSim.getPositionMeters() - hoodPositionOffset) * HOOD_METERS_TO_ROTATIONS;
    inputs.hoodCurrentAmps = hoodSim.getCurrentDrawAmps();
    inputs.hoodStalled = (inputs.hoodCurrentAmps > 8.0)
        && (Math.abs(hoodSim.getVelocityMetersPerSecond()) < 0.01);
    inputs.hoodAtSetpoint = Math.abs(inputs.hoodPositionRotations - hoodSetpoint) < Units.degreesToRotations(0.2);

    // Flywheel
    flywheelSim.setInput(flywheelAppliedVolts);
    flywheelSim.update(dt);
    inputs.flywheelVelocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(flywheelSim.getAngularVelocityRadPerSec());
    inputs.flywheelAtSetpoint = Math.abs(inputs.flywheelVelocityRPM - flywheelSetpointRPM) < 50.0;

    // Feeder
    feederSim.setInput(feederAppliedVolts);
    feederSim.update(dt);
    inputs.feederVelocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(feederSim.getAngularVelocityRadPerSec());

    // Spindexer
    spindexerSim.setInput(spindexerAppliedVolts);
    spindexerSim.update(dt);
    inputs.spindexerVelocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(spindexerSim.getAngularVelocityRadPerSec());
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    flywheelSetpointRPM = rpm;
    // NeoVortex free speed ~6784 RPM at 12V
    flywheelAppliedVolts = MathUtil.clamp((rpm / 6784.0) * 12.0, -12.0, 12.0);
  }

  @Override
  public void stopFlywheel(boolean coast) {
    flywheelSetpointRPM = 0.0;
    flywheelAppliedVolts = 0.0;
  }

  @Override
  public void setHoodPosition(double rotations) {
    hoodSetpoint = rotations;
    double currentRotations =
        (hoodSim.getPositionMeters() - hoodPositionOffset) * HOOD_METERS_TO_ROTATIONS;
    // P gain matched to hardware SparkMax kP = 0.1 (motor shaft rotations → duty cycle)
    hoodAppliedOutput = MathUtil.clamp((rotations - currentRotations) * 0.1, -1.0, 1.0);
  }

  @Override
  public void setHoodOutput(double output) {
    hoodAppliedOutput = output;
  }

  @Override
  public void resetHoodEncoder() {
    hoodPositionOffset = hoodSim.getPositionMeters();
    hoodSetpoint = 0.0;
  }

  @Override
  public void setFeederVelocity(double rpm) {
    // Neo 1.0 free speed ~5676 RPM at 12V
    feederAppliedVolts = MathUtil.clamp((rpm / 5676.0) * 12.0, -12.0, 12.0);
  }

  @Override
  public void setFeederOutput(double output) {
    feederAppliedVolts = output * 12.0;
  }

  @Override
  public void setSpindexerVelocity(double rpm) {
    // NeoVortex free speed ~6784 RPM at 12V
    spindexerAppliedVolts = MathUtil.clamp((rpm / 6784.0) * 12.0, -12.0, 12.0);
  }

  @Override
  public void setSpindexerOutput(double output) {
    spindexerAppliedVolts = output * 12.0;
  }
}
