// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

  // Mechanism2d: hood rack-and-pinion position mapped to 0–90° for display
  private static final double HOOD_DISPLAY_MAX_DEGREES = 90.0;
  private final LoggedMechanism2d m_mechanism = new LoggedMechanism2d(1, 1);
  private final LoggedMechanismRoot2d m_root = m_mechanism.getRoot("Shooter", 0.5, 0.1);
  private final LoggedMechanismLigament2d m_hood =
      m_root.append(new LoggedMechanismLigament2d("Hood", 0.35, 0, 6, new Color8Bit(Color.kOrange)));
  private final LoggedMechanismLigament2d m_hoodSetpoint =
      m_root.append(new LoggedMechanismLigament2d("HoodSetpoint", 0.35, 0, 2, new Color8Bit(Color.kDarkOrange)));

  public ShooterSubsystem(ShooterIO io, ShooterStateStore store) {
    this.io = io;
    this.stateStore = store;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    setState(stateStore.get());

    double scale = HOOD_DISPLAY_MAX_DEGREES / ShooterConstants.kHoodMaxAngle;
    m_hood.setAngle(inputs.hoodPositionRotations * scale);
    m_hoodSetpoint.setAngle(hoodSetpointRotations * scale);
    
    Logger.recordOutput("Shooter/Mechanism", m_mechanism);
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
    return Math.abs(inputs.flywheelVelocityRPM - flywheelSetpointRPM) < 50.0;
  }

  @AutoLogOutput(key = "Shooter/Ready")
  public boolean getReady() {
    return getIsAtSpeed() && inputs.hoodAtSetpoint;
  }

  /** Returns the total current draw of all shooter motors (amps). */
  public double getSimCurrentDrawAmps() {
    return inputs.hoodCurrentAmps + inputs.flywheelCurrentAmps
        + inputs.feederCurrentAmps + inputs.spindexerCurrentAmps;
  }
}
