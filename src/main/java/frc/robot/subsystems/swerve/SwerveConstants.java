package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.hardware.motors.PositionMotor;
import frc.robot.hardware.motors.VelocityMotor;
import frc.robot.utilities.FeedbackController;

import static frc.robot.WiringConstants.SwerveWiring.*;

public class SwerveConstants {

    public static final ChassisSpeeds MAX_SPEEDS = new ChassisSpeeds(4, 4, 4);
    public static final double MIN_COEFFICIENT = 0.2;
    public static final double MAX_MODULE_SPEED = 4;
    public static final TalonFXConfiguration DRIVE_CONFIG = 
        new TalonFXConfiguration()
            .withSlot1(new Slot1Configs()
                .withKP(0.11)
                .withKI(0.5)
                .withKD(0.0001)
                .withKV(0.12))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(35)
                .withSupplyCurrentLimitEnable(true)
            );
    public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(
        0.2974, 0.2974
    );
    public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(
        0.2974, -0.2974
    );
    public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(
        -0.2974, 0.2974
    );
    public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(
        -0.2974, -0.2974
    );

    public static final SwerveModule FRONT_LEFT_MODULE = new SwerveModule(
        VelocityMotor.fromTalonFX(
                FRONT_LEFT_DRIVE_ID, 
                motor -> {
                    TalonFXConfiguration config = new TalonFXConfiguration();
                    config.CurrentLimits = new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true);
                    config.MotorOutput = new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake);
                    config.Feedback = new FeedbackConfigs()
                        .withSensorToMechanismRatio(5.14 * Math.PI * 0.1016);
                    StatusCode status = StatusCode.StatusCodeNotInitialized;
                    for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                        status = motor.getConfigurator().apply(config);
                    }
                }, 
                FeedbackController.fromPID(
                    new PIDController(0.004, 0, 0),
                    controller -> {}
                ),
                new SimpleMotorFeedforward(0.055145, 0.19082, 0.0040725)
        ),
        PositionMotor.fromSparkMax(
            FRONT_LEFT_ANGLE_ID, 
            false,
            motor -> {
                motor.setSmartCurrentLimit(20);
                motor.getEncoder().setPositionConversionFactor(1.0 / 25);
                motor.getEncoder().setVelocityConversionFactor(1.0 / 25 / 60);
                motor.setIdleMode(IdleMode.kBrake);
            }, 
            FeedbackController.fromPID(
                new PIDController(27, 0, 0.5),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ), 
            new ElevatorFeedforward(0.18487, 0, 2.953, 0.22385)
        )
    );

    public static final SwerveModule FRONT_RIGHT_MODULE = new SwerveModule(
        VelocityMotor.fromTalonFX(
                FRONT_RIGHT_DRIVE_ID, 
                motor -> {
                    TalonFXConfiguration config = new TalonFXConfiguration();
                    config.CurrentLimits = new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true);
                    config.MotorOutput = new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake);
                    config.Feedback = new FeedbackConfigs()
                        .withSensorToMechanismRatio(5.14 * Math.PI * 0.1016);
                    StatusCode status = StatusCode.StatusCodeNotInitialized;
                    for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                        status = motor.getConfigurator().apply(config);
                    }
                }, 
                FeedbackController.fromPID(
                    new PIDController(0.004, 0, 0),
                    controller -> {}
                ),
                new SimpleMotorFeedforward(0.055145, 0.19082, 0.0040725)
        ),
        PositionMotor.fromSparkMax(
            FRONT_RIGHT_ANGLE_ID, 
            false,
            motor -> {
                motor.setSmartCurrentLimit(20);
                motor.getEncoder().setPositionConversionFactor(1.0 / 25);
                motor.getEncoder().setVelocityConversionFactor(1.0 / 25 / 60);
                motor.setIdleMode(IdleMode.kBrake);
            }, 
            FeedbackController.fromPID(
                new PIDController(27, 0, 0.5),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ), 
            new ElevatorFeedforward(0.18487, 0, 2.953, 0.22385)
        )
    );

    public static final SwerveModule BACK_LEFT_MODULE = new SwerveModule(
        VelocityMotor.fromTalonFX(
                BACK_LEFT_DRIVE_ID, 
                motor -> {
                    TalonFXConfiguration config = new TalonFXConfiguration();
                    config.CurrentLimits = new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true);
                    config.MotorOutput = new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake);
                    config.Feedback = new FeedbackConfigs()
                        .withSensorToMechanismRatio(5.14 * Math.PI * 0.1016);
                    StatusCode status = StatusCode.StatusCodeNotInitialized;
                    for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                        status = motor.getConfigurator().apply(config);
                    }
                }, 
                FeedbackController.fromPID(
                    new PIDController(0.004, 0, 0),
                    controller -> {}
                ),
                new SimpleMotorFeedforward(0.055145, 0.19082, 0.0040725)
        ),
        PositionMotor.fromSparkMax(
            BACK_LEFT_ANGLE_ID, 
            false,
            motor -> {
                motor.setSmartCurrentLimit(20);
                motor.getEncoder().setPositionConversionFactor(1.0 / 25);
                motor.getEncoder().setVelocityConversionFactor(1.0 / 25 / 60);
                motor.setIdleMode(IdleMode.kBrake);
            }, 
            FeedbackController.fromPID(
                new PIDController(27, 0, 0.5),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ), 
            new ElevatorFeedforward(0.18487, 0, 2.953, 0.22385)
        )
    );

    public static final SwerveModule BACK_RIGHT_MODULE = new SwerveModule(
        VelocityMotor.fromTalonFX(
                BACK_RIGHT_DRIVE_ID, 
                motor -> {
                    TalonFXConfiguration config = new TalonFXConfiguration();
                    config.CurrentLimits = new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true);
                    config.MotorOutput = new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake);
                    config.Feedback = new FeedbackConfigs()
                        .withSensorToMechanismRatio(5.14 * Math.PI * 0.1016);
                    StatusCode status = StatusCode.StatusCodeNotInitialized;
                    for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                        status = motor.getConfigurator().apply(config);
                    }
                }, 
                FeedbackController.fromPID(
                    new PIDController(0.004, 0, 0),
                    controller -> {}
                ),
                new SimpleMotorFeedforward(0.055145, 0.19082, 0.0040725)
        ),
        PositionMotor.fromSparkMax(
            BACK_RIGHT_ANGLE_ID, 
            false,
            motor -> {
                motor.setSmartCurrentLimit(20);
                motor.getEncoder().setPositionConversionFactor(1.0 / 25);
                motor.getEncoder().setVelocityConversionFactor(1.0 / 25 / 60);
                motor.setIdleMode(IdleMode.kBrake);
            }, 
            FeedbackController.fromPID(
                new PIDController(27, 0, 0.5),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ), 
            new ElevatorFeedforward(0.18487, 0, 2.953, 0.22385)
        )
    );
}
