// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.util.preferences.PreferenceNumber;


/** Add your docs here. */
public class SpeedSupplier implements DoubleSupplier {

    private PreferenceNumber normal = new PreferenceNumber("Drive/NormalMultiplier", DriveConstants.kNormalModeMultiplier);
    private PreferenceNumber slow = new PreferenceNumber("Drive/SlowMultiplier", DriveConstants.kSlowModeMultiplier);
    private PreferenceNumber fast = new PreferenceNumber("Drive/FastMultiplier", DriveConstants.kFastModeMultiplier);
    private PreferenceNumber intake = new PreferenceNumber("Drive/IntakeMultiplier", DriveConstants.kIntakeSpeedMultiplier);

    @AutoLogOutput(key = "Drive/CurrentMultiplier")
    private double speedMultiplier = normal.get();
    
    public double getAsDouble() {
        return speedMultiplier;
    }

    public void setSlow() {
        speedMultiplier = slow.get();
    }

    public void setFast() {
        speedMultiplier = fast.get();
    }

    public void reset() {
        speedMultiplier = normal.get();
    }

    public void setIntake() {
        speedMultiplier = intake.get();
    }
}
