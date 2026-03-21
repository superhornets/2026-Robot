// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class SpeedSupplier implements DoubleSupplier {
    @AutoLogOutput(key = "Robot/SpeedMultiplier")
    private double speedMultiplier = DriveConstants.kNormalModeMultiplier;
    
    public double getAsDouble() {
        return speedMultiplier;
    }

    public void setSlow() {
        speedMultiplier = DriveConstants.kSlowModeMultiplier;
    }

    public void setFast() {
        speedMultiplier = DriveConstants.kFastModeMultiplier;
    }

    public void reset() {
        speedMultiplier = DriveConstants.kNormalModeMultiplier;
    }
}
