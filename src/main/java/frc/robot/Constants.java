// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class FieldConstants {
    public static final Pose3d RedHubCenter =
        new Pose3d(Inches.of(469.11f), Inches.of(158.845f), Inches.of(72), Rotation3d.kZero);

    public static final Pose3d BlueHubCenter =
        new Pose3d(Inches.of(182.11f), Inches.of(158.845f), Inches.of(72), Rotation3d.kZero);

    public static final Pose3d blueLowerTrench =
        new Pose3d(Meters.of(4.6), Meters.of(0.6), Meters.of(0), Rotation3d.kZero);
    public static final Pose3d blueUpperTrench =
        new Pose3d(Meters.of(4.6), Meters.of(7.4), Meters.of(0), Rotation3d.kZero);
    public static final Pose3d redLowerTrench =
        new Pose3d(Meters.of(12), Meters.of(0.6), Meters.of(0), Rotation3d.kZero);
    public static final Pose3d redUpperTrench =
        new Pose3d(Meters.of(12), Meters.of(7.4), Meters.of(0), Rotation3d.kZero);
  }

  public static final class Robot {
    public static final double kWidthMeters = Units.inchesToMeters(20.5);
    public static final double kLengthMeters = Units.inchesToMeters(29.25);
  }

  public static final class DriveConstants {
    public static final double kSlowModeMultiplier = 0.25;
    public static final double kNormalModeMultiplier = 0.40; // 0.25 is the slow mode
    public static final double kFastModeMultiplier = 0.60;

    public static final double kWidthMeters = Units.inchesToMeters(20.5);
    public static final double kLengthMeters = Units.inchesToMeters(29.25);
  }

  public static final class Shooter {
    public static final class SIM {
      public static final double kFlywheelMOI = 0.0000117;
      public static final double kFlywheelGearRatio = 1.0;
      public static final double kHoodGearRatio = 1.0;
    }

    public static final double kFlywheelMaxSpeed = 6000.0; // RPM
    public static final double kFlywheelMinSpeed = -200.0; // RPM
    public static final double kHoodMinAngleDegrees = 0;
    public static final double kHoodMaxAngleDegrees = 90;

    public static final class CAN {
      public static final int kFlywheelLeft = 49;
      public static final int kFlywheelRight = 50;
      public static final int kFeeder = 51;
      public static final int kAgitator = 52;
      public static final int kHood = 53;
    }
  }

  public static final class Intake {
    public static final class SIM {
      public static final double kRollerMOI = 0.00117;
      public static final double kRollerGearRatio = 1.0;

      public static final double kArmLengthMeters = Units.inchesToMeters(14);
      public static final double kArmMOI =
          SingleJointedArmSim.estimateMOI(kArmLengthMeters, Units.lbsToKilograms(2));
      public static final double kArmGearRatio = 5.0;
    }

    public static final double kRaisedAngle = Units.degreesToRotations(0.0);
    public static final double kLoweredAngle = Units.degreesToRotations(90.0);
    public static final double kIntakeRollerSpeed = 2000.0; // RPM

    public static final class CAN {
      public static final int kLeftArm = 54;
      public static final int kLeftRoller = 55;
      public static final int kRightArm = 56;
      public static final int kRightRoller = 57;
    }
  }

  public static final class Climber {
    public static final class CAN {
      public static final int kClimber = 60;
    }

    // Physical limits for the climber arm (degrees)
    public static final double kMinAngleDegrees = 0.0;
    public static final double kMaxAngleDegrees = 90.0;

    // PID gains for position control (tune on robot)
    public static final double kPositionP = 1.0;
    public static final double kPositionI = 0.0;
    public static final double kPositionD = 0.1;

    public static final class SIM {
      // Motor rotations per arm rotation (gear reduction). Adjust to match your gearbox.
      public static final double kGearRatio = 100.0;

      // Simple arm model parameters used by simulation. Tune for better sim fidelity.
      public static final double kArmMassKg = 1.0;
      public static final double kArmLengthMeters = 1.0;
    }
  }

  public static final class SIM {
    public static final double interval = 1.0 / 50.0; // 50Hz
  }

}
