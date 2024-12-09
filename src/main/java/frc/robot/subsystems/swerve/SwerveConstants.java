package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.hardware.motors.PositionMotor;
import frc.robot.hardware.motors.VelocityMotor;
import frc.robot.utilities.FeedbackController;

import static frc.robot.WiringConstants.SwerveWiring.*;

public class SwerveConstants {

    public static final ChassisSpeeds MAX_SPEEDS = new ChassisSpeeds(6, 6, 4);
    public static final double MIN_COEFFICIENT = 0.2;
    public static final double MAX_MODULE_SPEED = 6;
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
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                    config.Feedback = new FeedbackConfigs()
                        .withSensorToMechanismRatio(12.1908);
                    StatusCode status = StatusCode.StatusCodeNotInitialized;
                    for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                        status = motor.getConfigurator().apply(config);
                    }
                }, 
                FeedbackController.fromPID(
                    new PIDController(0.004, 0, 0),
                    controller -> {}
                ),
                new SimpleMotorFeedforward(0.10624, 1.407, 0.16994) // Stolen from front right, sysID kA gain was wierd
        ),
        PositionMotor.fromSparkMax(
            FRONT_LEFT_ANGLE_ID, 
            false,
            motor -> {
                motor.setInverted(false);
                motor.setSmartCurrentLimit(20);
                motor.getEncoder().setPositionConversionFactor(1.0 / 25);
                motor.getEncoder().setVelocityConversionFactor(1.0 / 25 / 60);
                motor.setIdleMode(IdleMode.kBrake);
                AnalogEncoder absoluteEncoder = new AnalogEncoder(FRONT_LEFT_ENCODER_ID);
                motor.getEncoder().setPosition(absoluteEncoder.getAbsolutePosition() - 0.805);
                absoluteEncoder.close();
            }, 
            FeedbackController.fromPID(
                new PIDController(27, 0, 0),
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
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                    config.Feedback = new FeedbackConfigs()
                        .withSensorToMechanismRatio(12.1908);
                    StatusCode status = StatusCode.StatusCodeNotInitialized;
                    for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                        status = motor.getConfigurator().apply(config);
                    }
                }, 
                FeedbackController.fromPID(
                    new PIDController(0.004, 0, 0),
                    controller -> {}
                ),
                new SimpleMotorFeedforward(0.10624, 1.407, 0.16994)
        ),
        PositionMotor.fromSparkMax(
            FRONT_RIGHT_ANGLE_ID, 
            false,
            motor -> {
                motor.setInverted(false);
                motor.setSmartCurrentLimit(20);
                motor.getEncoder().setPositionConversionFactor(1.0 / 25);
                motor.getEncoder().setVelocityConversionFactor(1.0 / 25 / 60);
                motor.setIdleMode(IdleMode.kBrake);
                AnalogEncoder absoluteEncoder = new AnalogEncoder(FRONT_RIGHT_ENCODER_ID);
                motor.getEncoder().setPosition(absoluteEncoder.getAbsolutePosition() - 0.394);
                absoluteEncoder.close();
            }, 
            FeedbackController.fromPID(
                new PIDController(27, 0, 0),
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
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                    config.Feedback = new FeedbackConfigs()
                        .withSensorToMechanismRatio(12.1908);
                    StatusCode status = StatusCode.StatusCodeNotInitialized;
                    for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                        status = motor.getConfigurator().apply(config);
                    }
                }, 
                FeedbackController.fromPID(
                    new PIDController(0.004, 0, 0),
                    controller -> {}
                ),
                new SimpleMotorFeedforward(0.11833, 1.3984, 0.16306)
        ),
        PositionMotor.fromSparkMax(
            BACK_LEFT_ANGLE_ID, 
            false,
            motor -> {
                motor.setInverted(false);
                motor.setSmartCurrentLimit(20);
                motor.getEncoder().setPositionConversionFactor(1.0 / 25);
                motor.getEncoder().setVelocityConversionFactor(1.0 / 25 / 60);
                motor.setIdleMode(IdleMode.kBrake);
                AnalogEncoder absoluteEncoder = new AnalogEncoder(BACK_LEFT_ENCODER_ID);
                motor.getEncoder().setPosition(absoluteEncoder.getAbsolutePosition() - 0.511);
                absoluteEncoder.close();
            }, 
            FeedbackController.fromPID(
                new PIDController(27, 0, 0),
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
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                    config.Feedback = new FeedbackConfigs()
                        .withSensorToMechanismRatio(12.1908);
                    StatusCode status = StatusCode.StatusCodeNotInitialized;
                    for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                        status = motor.getConfigurator().apply(config);
                    }
                }, 
                FeedbackController.fromPID(
                    new PIDController(0.004, 0, 0),
                    controller -> {}
                ),
                new SimpleMotorFeedforward(0.10512, 1.3954, 0.19264)
        ),
        PositionMotor.fromSparkMax(
            BACK_RIGHT_ANGLE_ID, 
            false,
            motor -> {
                motor.setInverted(false);
                motor.setSmartCurrentLimit(20);
                motor.getEncoder().setPositionConversionFactor(1.0 / 25);
                motor.getEncoder().setVelocityConversionFactor(1.0 / 25 / 60);
                motor.setIdleMode(IdleMode.kBrake);
                AnalogEncoder absoluteEncoder = new AnalogEncoder(BACK_RIGHT_ENCODER_ID);
                motor.getEncoder().setPosition(absoluteEncoder.getAbsolutePosition() - 0.061);
                absoluteEncoder.close();
            }, 
            FeedbackController.fromPID(
                new PIDController(27, 0, 0),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ), 
            new ElevatorFeedforward(0.18487, 0, 2.953, 0.22385)
        )
    );
}
