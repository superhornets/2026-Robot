// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.shooter.ShooterState;

/** Add your docs here. */
public class RebuiltField {
    
    private static BooleanSupplier isFlippedSupplier = () -> { return false; };
    private static Supplier<Pose2d> robotPoseSupplier = () -> { return new Pose2d(); };

    public static void setGlobalFlippedSupplier(BooleanSupplier newFlippedSupplier) {
        isFlippedSupplier = newFlippedSupplier;
    }

    public static void setGlobalRobotPoseSupplier(Supplier<Pose2d> newSupplier) {
        robotPoseSupplier = newSupplier;
    }

    public static boolean getIsFlipped() {
        return isFlippedSupplier.getAsBoolean();
    }

    public static Pose3d getHubCenter() {
        return getIsFlipped()
            ? Constants.FieldConstants.RedHubCenter
            : Constants.FieldConstants.BlueHubCenter;
    }

    public static Translation2d getTranslationToHub2D() {
        Pose3d hubCenter = getHubCenter();
        Pose2d pose = robotPoseSupplier.get();

        double relativeHubX = hubCenter.getX() - pose.getX();
        double relativeHubY = hubCenter.getY() - pose.getY();

        return new Translation2d(relativeHubX, relativeHubY);
    }

    public static ShooterState shooterStateForHub() {
        // minimum shooting distance
        double minDist = 1.32;
        ShooterState min = new ShooterState(37, 0);

        // Maximum shooting distance
        double maxDist = 5.21;
        ShooterState max = new ShooterState(49, 21);

        // get the distance from the hub
        double distance = getTranslationToHub2D().getNorm();
        // calculate the ratio between our min and max that we are at
        double ratio = InverseInterpolator.forDouble().inverseInterpolate(minDist, maxDist, distance);

        // interpolate our shooter configuration between the min and max based on the ratio
        return min.interpolate(max, ratio);
    }
}
