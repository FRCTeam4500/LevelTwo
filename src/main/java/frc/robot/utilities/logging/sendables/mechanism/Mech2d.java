package frc.robot.utilities.logging.sendables.mechanism;

import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

import java.util.HashMap;

public class Mech2d implements Loggable {
    private double[] dims;
    private HashMap<String, MechRoot2d> roots;
    public Mech2d(double width, double height) {
        dims = new double[] {width, height};
        roots = new HashMap<>();
    }

    public MechRoot2d getRoot(String name, double x, double y) {
        if (roots.containsKey(name)) {
            return roots.get(name);
        }
        MechRoot2d root = new MechRoot2d(x, y);
        roots.put(name, root);
        return root;

    }

    @Override
    public void log(String name) {
        HoundLog.log(name + "/.type", "Mechanism2d");
        HoundLog.log(name + "/backgroundColor", "#00020");
        HoundLog.log(name + "/.controllable", true);
        HoundLog.log(name + "/dims", dims);
        for (String rootName : roots.keySet()) {
            roots.get(rootName).log(name + "/" + rootName);
        }
    }
}
