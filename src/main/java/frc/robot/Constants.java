package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Constants {
        public static class Vision {
 
        public static final String kHotSauceCamera = "hot_sauce";
        public static final String kMildSauceCamera = "mild_sauce";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToHotSauce =
                new Transform3d(new Translation3d(0.42545, 0.0, 0.13335), new Rotation3d(0, -Math.PI / 3, 0));
        public static final Transform3d kRobotToMildSauce =
                new Transform3d(new Translation3d(-0.42545, 0.12065, 0.13335), new Rotation3d(0, -Math.PI / 3, Math.PI));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
