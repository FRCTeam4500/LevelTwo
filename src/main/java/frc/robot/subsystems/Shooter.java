package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.motors.PositionMotor;
import frc.robot.hardware.motors.VelocityMotor;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.Loggable;

import static frc.robot.WiringConstants.ShooterWiring.*;

import com.revrobotics.CANSparkBase.IdleMode;

public class Shooter extends SubsystemBase implements Loggable {
    private PositionMotor tiltMotor;
    private VelocityMotor rightMotor;
    private VelocityMotor leftMotor;
    private VelocityMotor loaderMotor;

    public Shooter() {
        tiltMotor = PositionMotor.fromSparkMax(
            TILT_ID, 
            false, 
            motor -> {
                motor.setInverted(false);
                motor.getEncoder().setPositionConversionFactor(1.0 / 70);
                motor.getEncoder().setVelocityConversionFactor(1.0 / 70 / 60);
                motor.setSmartCurrentLimit(30);
                motor.setIdleMode(IdleMode.kCoast);
            }, 
            FeedbackController.fromPID(
                new PIDController(3, 0, 0), 
                controller -> {
                    controller.setTolerance(0.025);
                }
            ), 
            null
        );
        rightMotor = VelocityMotor.fromSparkMax(
            RIGHT_ID, 
            false, 
            motor -> {
                motor.setInverted(false);
                motor.setSmartCurrentLimit(30);
                motor.setIdleMode(IdleMode.kCoast);
            }, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0), 
                pid -> {}
            ), 
            new SimpleMotorFeedforward(0.4128, 0.0022348, 0.00022425)
        );
        leftMotor = VelocityMotor.fromSparkMax(
            LEFT_ID, 
            false, 
            motor -> {
                motor.setInverted(false);
                motor.setSmartCurrentLimit(30);
                motor.setIdleMode(IdleMode.kCoast);
            }, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0), 
                pid -> {}
            ), 
            new SimpleMotorFeedforward(0.43436, 0.002232, 0.00020249)
        );
        loaderMotor = VelocityMotor.fromSparkMax(
            LOADER_ID, 
            false, 
            motor -> {
                motor.setInverted(false);
                motor.setSmartCurrentLimit(40);
                motor.setIdleMode(IdleMode.kBrake);
            }, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0), 
                pid -> {}
            ), 
            new SimpleMotorFeedforward(0.45067, 0.0022803, 0.00013867)
        );
    }

    @Override
    public void log(String name) {
        tiltMotor.log(name + "/Tilt Motor");
        rightMotor.log(name + "/Right Shooter Motor");
        leftMotor.log(name + "/Left Shooter Motor");
        loaderMotor.log(name + "/Loader Motor");
    }

    public Command startIntake() {
        return runOnce(
            () -> {
                tiltMotor.setTarget(0);
                rightMotor.setTarget(-2000);
                leftMotor.setTarget(-2000);
                loaderMotor.setTarget(0);
            }
        ).andThen(
            Commands.waitUntil(() -> allAtTarget())
        );
    }

    public Command finishIntake() {
        return runOnce(
            () -> {
                tiltMotor.setTarget(0.1);
                rightMotor.setTarget(-2000);
                leftMotor.setTarget(-2000);
                loaderMotor.setTarget(0);
            }
        ).andThen(
            Commands.waitUntil(() -> tiltMotor.atTarget())
        ).andThen(
            runOnce(
                () -> {
                    loaderMotor.setTarget(-2000);
                }
            )
        ).andThen(
            Commands.waitSeconds(0.1)
        ).andThen(
            stow()
        );
    }

    public Command readyAmp() {
        return runOnce(
            () -> {
                tiltMotor.setTarget(0.25);
                rightMotor.setTarget(2000);
                leftMotor.setTarget(2000);
                loaderMotor.setTarget(0);
            }
        ).andThen(
            Commands.waitUntil(() -> allAtTarget())
        );
    }

    public Command fire() {
        return runOnce(
            () -> {
                loaderMotor.setTarget(2000);
            }
        ).andThen(
            Commands.waitSeconds(1)
        ).andThen(
            runOnce(
                () -> {
                    loaderMotor.setTarget(0);
                }
            )
        );
    }

    public Command stow() {
        return runOnce(
            () -> {
                tiltMotor.setTarget(0);
                rightMotor.setTarget(0);
                leftMotor.setTarget(0);
                loaderMotor.setTarget(0);
            }
        ).andThen(
            Commands.waitUntil(() -> allAtTarget())
        );
    }

    private boolean allAtTarget() {
        return tiltMotor.atTarget() &&
            loaderMotor.atTarget() &&
            (rightMotor.atTarget() || leftMotor.atTarget());
    }


}
