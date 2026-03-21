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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

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
}
