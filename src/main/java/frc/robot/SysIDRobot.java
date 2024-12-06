package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.motors.VelocityMotor;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.SysIDCommands;

import static frc.robot.WiringConstants.SwerveWiring.*;

public class SysIDRobot extends TimedRobot {
    private VelocityMotor fl = VelocityMotor.fromTalonFX(
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
            new SimpleMotorFeedforward(0.055145, 0.19082, 0.0040725)
    );
    private VelocityMotor fr = VelocityMotor.fromTalonFX(
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
        new SimpleMotorFeedforward(0.055145, 0.19082, 0.0040725)
    );
    private VelocityMotor bl = VelocityMotor.fromTalonFX(
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
        new SimpleMotorFeedforward(0.055145, 0.19082, 0.0040725)
    );
    private VelocityMotor br = VelocityMotor.fromTalonFX(
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
        new SimpleMotorFeedforward(0.055145, 0.19082, 0.0040725)
);

    public SysIDRobot() {
        int[] ids = new int[] {FRONT_LEFT_ANGLE_ID, FRONT_RIGHT_ANGLE_ID, BACK_LEFT_ANGLE_ID, BACK_RIGHT_ANGLE_ID};
        for (int id : ids) {
            CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
            motor.setIdleMode(IdleMode.kBrake);
            motor.close();
        }
        SysIDCommands commands = fl.getSynchronizedSysID(
            "Swerve", 
            0.5, 
            1, 
            5, 
            fr, bl, br
        );
        SmartDashboard.putData("Dynamic Forward", commands.dynamicForward());
        SmartDashboard.putData("Dynamic Reverse", commands.dynamicReverse());
        SmartDashboard.putData("Quasistatic Forward", commands.quasistaticForward());
        SmartDashboard.putData("Quasistatic Reverse", commands.quasistaticReverse());

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
