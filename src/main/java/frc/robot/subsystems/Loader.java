package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.motors.VelocityMotor;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.Loggable;

import static frc.robot.WiringConstants.LoaderWiring.*;

import com.revrobotics.CANSparkBase.IdleMode;

public class Loader extends SubsystemBase implements Loggable {
    private VelocityMotor motor;

    public Loader() {
        motor = VelocityMotor.fromSparkmax(
            MOTOR_ID, 
            false, 
            motor -> {
                motor.setIdleMode(IdleMode.kBrake);
                motor.setSmartCurrentLimit(40);
            }, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0), 
                controller -> {
                    controller.setTolerance(10);
                }
            ), 
            new SimpleMotorFeedforward(0, 0, 0));
    }

    @Override
    public void log(String name) {
        motor.log(name + "/Motor");
    }

    public Command take() {
        return run(() -> {
            motor.gotoVelocity(300);
        }).withTimeout(0.1);
    }

    public Command fire() {
        return run(() -> {
            motor.gotoVelocity(-300);
        }).withTimeout(1);
    }

    public Command eject() {
        return run(() -> {
            motor.gotoVelocity(1000);
        }).withTimeout(0.5);
    }

}
