package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.logging.Loggable;

public class Elevator extends SubsystemBase implements Loggable {
    // TODO: Make a PositionMotor and initalize it in the Elevator Constructor

    @Override
    public void log(String name) {
        // TODO: Log the PositionMotor you made
    }

    public Command extend() {
        // TODO: Change this command to one that sets the PositionMotor's target to 
        return Commands.none();
    }

    public Command stow() {
        // TODO: Change this command to one that sets the PositionMotor's target to 0
        return Commands.none();
    }
}
