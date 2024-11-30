package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Loader;
import frc.robot.utilities.logging.Loggable;
import frc.robot.utilities.logging.sendables.mechanism.Mech2d;

public class Superstructure implements Loggable {
    // Create objects for all non-drivebase subsystems
    private Elevator elevator;
    private Loader loader;
    private Mech2d robotMech;
    public Superstructure() {
        elevator = new Elevator();
        loader = new Loader();
        robotMech = new Mech2d(1.5, 1.5);
        configureMech();
    }

    private void configureMech() {
        // Append subsystem mechs
    }

    public void log(String name) {
        // Call log() methods for contained subsystems
        robotMech.log("Robot Mech");
        elevator.log("Elevator");
    }

    // Put Command Factories Here

    public Command exampleFactory() {
        return elevator.extend().alongWith(
            Commands.print("Arm is extending!").andThen(
                Commands.waitUntil(elevator.atHeight)
            ).andThen(
                loader.eject()
            )
        );
    }
}
