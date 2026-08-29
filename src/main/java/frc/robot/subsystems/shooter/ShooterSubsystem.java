// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterStateStore stateStore;
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Output setpoints — tracked here so they can be logged without reading back from hardware
  private double flywheelSetpointRPM = 0.0;
  private double hoodSetpointRotations = 0.0;
  private double feederSetpointRPM = 0.0;

  public ShooterSubsystem(ShooterIO io, ShooterStateStore store) {
    this.io = io;
    this.stateStore = store;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    setState(stateStore.get());
  }

  public void setState(ShooterState state) {
    setHoodAngle(state.angle);
    startFlywheel(state.speed);
  }

  public void setHoodAngle(double angleRotations) {
    double value = Math.max(ShooterConstants.kHoodMinAngle,
        Math.min(ShooterConstants.kHoodMaxAngle, angleRotations));
    hoodSetpointRotations = value;
    io.setHoodPosition(value);
  }

  public void startFlywheel(double speedRPM) {
    double speedClamped = MathUtil.clamp(
        speedRPM, ShooterConstants.kFlywheelMinSpeed, ShooterConstants.kFlywheelMaxSpeed);
    flywheelSetpointRPM = speedClamped;
    io.setFlywheelVelocity(speedClamped);
  }

  public void stopFlywheel(boolean coast) {
    flywheelSetpointRPM = 0.0;
    io.stopFlywheel(coast);
  }

  public void startFeeder() {
    feederSetpointRPM = 4000;
    io.setFeederVelocity(4000);
    io.setSpindexerVelocity(6000);
  }

  public void startReverseFeeder() {
    feederSetpointRPM = 0.0;
    io.setFeederOutput(0.0);
    io.setSpindexerVelocity(-2000);
  }

  public void stopFeeder() {
    feederSetpointRPM = 0.0;
    io.setFeederOutput(0.0);
    io.setSpindexerOutput(0.0);
  }

  public void startHoodZeroing() {
    io.setHoodOutput(-0.10);
  }

  public void stopHoodZeroing() {
    io.setHoodOutput(0.0);
  }

  public boolean isHoodStalled() {
    return inputs.hoodStalled;
  }

  public void setHoodZero() {
    io.setHoodOutput(0.0);
    io.resetHoodEncoder();
    hoodSetpointRotations = 0.0;
    io.setHoodPosition(0.0);
  }

  @AutoLogOutput(key = "Shooter/hoodAngle")
  public double getHoodAngle() {
    return inputs.hoodPositionRotations;
  }

  @AutoLogOutput(key = "Shooter/hoodAngleSetpoint")
  public double getHoodAngleSet() {
    return hoodSetpointRotations;
  }

  @AutoLogOutput(key = "Shooter/FlywheelVelocitySetpointRPM")
  public double getFlywheelVelocitySetpointRPM() {
    return flywheelSetpointRPM;
  }

  @AutoLogOutput(key = "Shooter/FlywheelVelocityRPM")
  public double getFlywheelVelocityRPM() {
    return inputs.flywheelVelocityRPM;
  }

  @AutoLogOutput(key = "Shooter/FeederSetpointRPM")
  public double getFeederSetpointRPM() {
    return feederSetpointRPM;
  }

  @AutoLogOutput(key = "Shooter/isAtSpeed")
  public boolean getIsAtSpeed() {
    return inputs.flywheelAtSetpoint;
  }

  @AutoLogOutput(key = "Shooter/Ready")
  public boolean getReady() {
    return inputs.flywheelAtSetpoint && inputs.hoodAtSetpoint;
  }
}
