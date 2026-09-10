// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.util.preferences.PreferenceNumber;


/** Add your docs here. */
public class ShooterStateStore implements Supplier<ShooterState> {

    private ShooterState current = ShooterState.zero;
    private ShooterState last = ShooterState.zero;

    private PreferenceNumber hoodAdjustment = new PreferenceNumber("Shooter/HoodIncrement", .6);
    private PreferenceNumber speedAdjustment = new PreferenceNumber("Shooter/SpeedIncrement", 100);
    private DoubleSupplier idleSupplier = ShooterState.flywheelIdleSpeed;


    public ShooterState get() {
        return current;
    }

    public void set(ShooterState updated) {
        current = updated.clamped();
    }

    public void stop() {
        // store the last position so that we can return there if we want to
        last = current;
        set(ShooterState.zero);
    }

    public ShooterState getIdle() {
        return new ShooterState(idleSupplier.getAsDouble(), 0);
    }

    public void idle() {
        // store the last position so that we can return there if we want to
        last = current;
        set(getIdle());
    }

    public boolean isIdle() {
        return current.equals(getIdle()) || current.equals(ShooterState.zero);
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
