package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.logging.Loggable;

public class Superstructure implements Loggable {
    private Shooter shooter;
    private Elevator elevator;
    public Superstructure() {
        shooter = new Shooter();
        elevator = new Elevator();
    }

    public void log(String name) {
        shooter.log(name + "/Shooter");
        elevator.log(name + "/Elevator");
    }

    public Command startIntake() {
        return elevator.stow()
            .alongWith(shooter.startIntake());
    }

    public Command finishIntake() {
        return elevator.stow()
            .alongWith(shooter.finishIntake());
    }

    public Command readyAmp() {
        return elevator.extend()
            .alongWith(shooter.readyAmp());
    }

    public Command fire() {
        return shooter.fire()
            .andThen(stow());
    }

    public Command stow() {
        return shooter.stow()
            .alongWith(elevator.stow());
    }
}
