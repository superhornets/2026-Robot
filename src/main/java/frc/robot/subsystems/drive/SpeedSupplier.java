// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


/** Add your docs here. */
public class SpeedSupplier implements DoubleSupplier {

    private LoggedNetworkNumber normal = new LoggedNetworkNumber("Drive/NormalMultiplier", DriveConstants.kNormalModeMultiplier);
    private LoggedNetworkNumber slow = new LoggedNetworkNumber("Drive/SlowMultiplier", DriveConstants.kSlowModeMultiplier);
    private LoggedNetworkNumber fast = new LoggedNetworkNumber("Drive/FastMultiplier", DriveConstants.kFastModeMultiplier);
    private LoggedNetworkNumber intake = new LoggedNetworkNumber("Drive/IntakeMultiplier", DriveConstants.kIntakeSpeedMultiplier);

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
