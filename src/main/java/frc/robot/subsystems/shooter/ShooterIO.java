// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double flywheelVelocityRPM = 0.0;
    public boolean flywheelAtSetpoint = false;

    public double hoodPositionRotations = 0.0;
    public boolean hoodAtSetpoint = false;
    public double hoodCurrentAmps = 0.0;
    public boolean hoodStalled = false;

    public double feederVelocityRPM = 0.0;
    public double spindexerVelocityRPM = 0.0;
  }

  /** Called every loop to populate the logged inputs struct from hardware/sim. */
  default void updateInputs(ShooterIOInputs inputs) {}

  // --- Flywheel ---
  default void setFlywheelVelocity(double rpm) {}

  default void stopFlywheel(boolean coast) {}

  // --- Hood ---
  default void setHoodPosition(double rotations) {}

  /** Direct percent-output drive, used during zeroing. */
  default void setHoodOutput(double output) {}

  /** Reset the hood encoder to zero (call after zeroing to hard stop). */
  default void resetHoodEncoder() {}

  // --- Feeder / Spindexer ---
  default void setFeederVelocity(double rpm) {}

  default void setFeederOutput(double output) {}

  default void setSpindexerVelocity(double rpm) {}

  default void setSpindexerOutput(double output) {}
}
