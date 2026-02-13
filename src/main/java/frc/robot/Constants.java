// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

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
  }

  public static final class DriveConstants {

    public static final double kSlowModeMultiplier = 0.25;

    public static final double kFastModeMultiplier = 0.60;
    public static final double kNormalModeMultiplier = 0.40; // 0.25 is the slow mode
  }

  public static final class CanIds {
    public static final int ClimberMotor1 = 89;
  }
}
