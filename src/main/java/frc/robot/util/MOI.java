package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class MOI {
  public static double in2lbToKgM2(double in2lbT) {
    return Units.inchesToMeters(Units.inchesToMeters(Units.lbsToKilograms(in2lbT)));
  }
}
