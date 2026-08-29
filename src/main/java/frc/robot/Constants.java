// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.util.Units;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;


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
    public static final Pose3d center = new Pose3d(Meters.of((12.0 + 4.6) * 0.5), Meters.of((0.6 + 7.4) * 0.5), Meters.of(0), Rotation3d.kZero);
  }

  public static final class Robot {
    public static final double kWidthMeters = Units.inchesToMeters(20.5);
    public static final double kLengthMeters = Units.inchesToMeters(29.25);
  }

  /** CAN IDs and port assignments for the shooter mechanism. */
  public static final class Shooter {
    public static final class CAN {
      public static final int kFlywheelLeft = 49;
      public static final int kFlywheelRight = 50;
      public static final int kFeeder = 51;
      public static final int kAgitator = 52;
      public static final int kSpindexer = 59;
      public static final int kHood = 53;
    }
  }

  /** CAN IDs and port assignments for the intake mechanism. */
  public static final class Intake {
    public static final class CAN {
      public static final int kRightArm = 56;
      public static final int kRightRoller = 57;
    }

    public static final int kServo1Port = 8;
    public static final int kServo2Port = 9;
  }

public static final class SIM {
    public static final double interval = 1.0 / 50.0; // 50Hz
  }
}
