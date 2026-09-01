// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

public final class DriveConstants {
  public static final double kSlowModeMultiplier = 0.25;
  public static final double kNormalModeMultiplier = 0.7;
  public static final double kFastModeMultiplier = 0.9;
  public static final double kIntakeSpeedMultiplier = 0.2;

  /** Angular tolerance (degrees) for declaring the robot aimed at the hub. */
  public static final double kHubAlignmentToleranceDegrees = 2.0;
}
