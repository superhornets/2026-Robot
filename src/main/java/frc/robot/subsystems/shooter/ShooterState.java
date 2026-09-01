// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

/** Add your docs here. */
public class ShooterState implements Interpolatable<ShooterState> {

    public static final ShooterState zero = new ShooterState(0, 0);
    public static final ShooterState min = new ShooterState(ShooterConstants.kFlywheelMinSpeed, 0);
    public static final ShooterState max = new ShooterState(ShooterConstants.kFlywheelMaxSpeed, 30);
    public static final ShooterState idle = new ShooterState(ShooterConstants.kFlywheelIdleSpeed, 30);
    public static final ShooterState reverse = new ShooterState(ShooterConstants.kFlywheelMinSpeed, 0);

    public double speed = 0.0;
    public double angle = 0.0;

    public ShooterState(double speed, double angle) {
        this.speed = speed;
        this.angle = angle;
    }

    public ShooterState interpolate(ShooterState endValue, double t) {
        return new ShooterState(
            speed + ((endValue.speed - speed) * t),
            angle + ((endValue.angle - angle) * t)
            );
    }

    public ShooterState clamped() {
        return new ShooterState(
            MathUtil.clamp(speed, ShooterState.min.speed, ShooterState.max.speed), 
            MathUtil.clamp(angle, ShooterState.min.angle, ShooterState.max.angle));
    }
}
