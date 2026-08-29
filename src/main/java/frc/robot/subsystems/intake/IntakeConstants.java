// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.util.MOI;

public final class IntakeConstants {
  public static final double kRaisedAngle = 0.2;
  public static final double kLoweredAngle = 0.5;
  public static final double kIntakeRollerSpeedLeft = 2000.0; // RPM
  public static final double kIntakeRollerSpeedRight = 4000.0; // RPM

  public static final class SIM {
    public static final double kRollerMOI = MOI.in2lbToKgM2(.5);
    public static final double kRollerGearRatio = 1.0;
    public static final double kArmLengthMeters = Units.inchesToMeters(14);
    public static final double kArmMOI = MOI.in2lbToKgM2(350);
    public static final double kArmGearRatio = 45.0;
  }
}
