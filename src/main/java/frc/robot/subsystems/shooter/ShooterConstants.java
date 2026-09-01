// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import frc.robot.util.MOI;

public final class ShooterConstants {
  public static final double kFlywheelMaxSpeed = 4500.0; // RPM
  public static final double kFlywheelMinSpeed = -200.0; // RPM
  public static final double kFlywheelIdleSpeed = 2000.0; // RPM — warm idle, spins up quickly
  public static final double kHoodMinAngle = 0;
  public static final double kHoodMaxAngle = 25;

  public static final class SIM {
    public static final double kFlywheelMOI = MOI.in2lbToKgM2(10.7);
    public static final double kFlywheelGearRatio = 1.0;

    public static final double kHoodGearRatio = 9.0; // 9:1 gearbox

    // Feeder: Neo 1.0, 4:1 gear ratio. MOI is an estimate — update with measured value.
    public static final double kFeederMOI = kFlywheelMOI * 0.3;
    public static final double kFeederGearRatio = 4.0;

    // Spindexer: NeoVortex (SparkFlex), 4:1 gear ratio, 2 in²lb MOI
    public static final double kSpindexerMOI = MOI.in2lbToKgM2(2.0);
    public static final double kSpindexerGearRatio = 4.0;
  }
}
