// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


/** Add your docs here. */
public class ShooterStateStore implements Supplier<ShooterState> {

    private ShooterState current = ShooterState.zero;

    private ShooterState last = ShooterState.zero;

    private LoggedNetworkNumber hoodAdjustment = new LoggedNetworkNumber("Shooter/HoodAdjustment", .6);
    private LoggedNetworkNumber speedAdjustment = new LoggedNetworkNumber("Shooter/SpeedAdjustment", 100);

    public ShooterState get() {
        return current;
    }

    public void set(ShooterState updated) {
        current = updated.clamped();
    }

    public void reset() {
        // store the last position so that we can return there if we want to
        last = current;
        current = ShooterState.min;
    }

    public void returnToLast() {
        current = last;
    }

    public void modifyAngle(double adjustment) {
        current.angle += adjustment * hoodAdjustment.get();
        current = current.clamped();
    }

    public void modifySpeed(double adjustment) {
        current.speed += adjustment * speedAdjustment.get();
        current = current.clamped();
    }

}
