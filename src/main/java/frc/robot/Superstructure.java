package frc.robot;

import frc.robot.utilities.logging.Loggable;
import frc.robot.utilities.logging.sendables.mechanism.Mech2d;

public class Superstructure implements Loggable {
    // Create objects for all non-drivebase subsystems
    private Mech2d robotMech;
    public Superstructure() {
        robotMech = new Mech2d(1.5, 1.5);
        configureMech();
    }

    private void configureMech() {
        // Append subsystem mechs
    }

    public void log(String name) {
        // Call log() methods for contained subsystems
        robotMech.log("Robot Mech");
    }

    // Put Command Factories Here
}
