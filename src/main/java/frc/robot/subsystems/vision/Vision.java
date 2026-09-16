// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final DoubleFunction<Optional<Pose2d>> poseAtTimestamp;
  private final DoubleSupplier angularVelocityRadPerSec;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(
      VisionConsumer consumer,
      DoubleFunction<Optional<Pose2d>> poseAtTimestamp,
      DoubleSupplier angularVelocityRadPerSec,
      VisionIO... io) {
    this.consumer = consumer;
    this.poseAtTimestamp = poseAtTimestamp;
    this.angularVelocityRadPerSec = angularVelocityRadPerSec;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();
      List<String> rejectionReasons = new LinkedList<>();
      double angularVelocity = Math.abs(angularVelocityRadPerSec.getAsDouble());

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        String rejectionReason = getRejectionReason(observation, angularVelocity);
        boolean rejectPose = !rejectionReason.isEmpty();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          rejectionReasons.add(rejectionReason);
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }
        double motionStdDevFactor = 1.0 + angularVelocity * motionStdDevFactorPerRadPerSec;
        linearStdDev *= motionStdDevFactor;
        angularStdDev *= motionStdDevFactor;

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RejectionReasons",
          rejectionReasons.toArray(new String[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/AngularVelocityRadPerSec",
          angularVelocity);
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  /** Returns why an observation is unsafe to fuse, or an empty string when it is acceptable. */
  private String getRejectionReason(VisionIO.PoseObservation observation, double angularVelocity) {
    if (observation.tagCount() == 0) return "NoTags";
    if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) return "Ambiguous";
    if (Math.abs(observation.pose().getZ()) > maxZError) return "InvalidZ";
    if (observation.pose().getX() < 0.0 || observation.pose().getX() > aprilTagLayout.getFieldLength()
        || observation.pose().getY() < 0.0 || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
      return "OutsideField";
    }

    // AdvantageKit's timestamp advances with replay logs as well as on a live robot.
    double observationAgeSeconds = Logger.getTimestamp() / 1.0e6 - observation.timestamp();
    if (observationAgeSeconds < 0.0 || observationAgeSeconds > maxObservationAgeSeconds) return "Stale";
    if (observation.tagCount() == 1 && angularVelocity > maxSingleTagAngularVelocityRadPerSec) {
      return "SingleTagHighRotation";
    }

    Optional<Pose2d> estimatedPose = poseAtTimestamp.apply(observation.timestamp());
    if (estimatedPose.isEmpty()) return "NoOdometrySample";
    if (estimatedPose.get().getTranslation().getDistance(observation.pose().toPose2d().getTranslation())
        > maxPoseTranslationErrorMeters) return "TranslationOutlier";
    if (observation.type() != PoseObservationType.MEGATAG_2
        && Math.abs(estimatedPose.get().getRotation().minus(observation.pose().toPose2d().getRotation()).getRadians())
            > maxPoseRotationErrorRadians) return "RotationOutlier";
    return "";
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
