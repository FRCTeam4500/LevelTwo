package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.motors.PositionMotor;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.Loggable;

import static frc.robot.WiringConstants.ElevatorWiring.*;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class Elevator extends SubsystemBase implements Loggable {
    private PositionMotor heightMotor;
    public final Trigger atHeight;

    public Elevator() {
        heightMotor = PositionMotor.fromTalonSRX(
            HEIGHT_ID, 
            1 / 25826.7716535, 
            motor -> {
                motor.setInverted(false);
                motor.setSelectedSensorPosition(0);
                motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 31, 0.1));
            }, 
            FeedbackController.fromPID(
                new PIDController(100, 0, 0), 
                controller -> {
                    controller.setTolerance(0.01);
                    controller.calculate(0, 0);
                }
            ), 
            null
        );
        atHeight = new Trigger(heightMotor::atTarget);
    }

    @Override
    public void log(String name) {
        heightMotor.log(name + "/Height Motor");
    }

    public Command extend() {
        return run(() -> heightMotor.gotoPosition(0.1));
    }

    public Command stow() {
        return run(() -> heightMotor.gotoPosition(0));
    }
    
}
