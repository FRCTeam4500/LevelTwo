package frc.robot.utilities.logging.sendables;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class LoggedField2d implements Loggable {
    private HashMap<String, double[]> objects;
    public LoggedField2d() {
        objects = new HashMap<>();
        objects.put("Robot", new double[] {0, 0, 0});
    }

    public void setRobotPose(Pose2d pose) {
        setObjectPose("Robot", pose);
    }

    public void setObjectPose(String name, Pose2d pose) {
        objects.put(name, new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    }

    @Override
    public void log(String name) {
        HoundLog.log(name + "/.controllable", true);
        HoundLog.log(name + "/.type", "Field2d");
        HoundLog.log(name + "/.name", name);
        for (String object : objects.keySet()) {
            HoundLog.log(name + "/" + object, objects.get(object));
        }
    }
}
