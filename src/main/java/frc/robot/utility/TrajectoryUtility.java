package frc.robot.utility;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryUtility {
    public static Trajectory createNewTrajectoryFromJSON(String filename) {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("output/" + filename);
        try {
            return TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }
}
