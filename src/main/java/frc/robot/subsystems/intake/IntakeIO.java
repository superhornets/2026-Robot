// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public double armPositionRotations = 0.0;
    public boolean armAtSetpoint = false;
    public double rollerVelocityRPM = 0.0;
  }

  /** Called every loop to populate the logged inputs struct from hardware/sim. */
  default void updateInputs(IntakeIOInputs inputs) {}

  // --- Arm ---
  default void setArmPosition(double rotations) {}

  // --- Servos ---
  default void setServoAngles(double angle1, double angle2) {}

  // --- Roller ---
  /** Spin the roller at the given speed in RPM (negative = reverse). */
  default void setRollerVelocity(double rpm) {}

  default void stopRoller() {}
}
