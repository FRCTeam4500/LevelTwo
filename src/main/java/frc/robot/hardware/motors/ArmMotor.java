package frc.robot.hardware.motors;

import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public interface ArmMotor extends Loggable {
    public void setRotations(double target);
    public void resetRotations(double actual);
    public double getRotations();

    public static ArmMotor fromTalonFX(
        int canID,
        Consumer<TalonFX> config,
        FeedbackController fb,
        ArmFeedforward ff
    ) {
        if (RobotBase.isSimulation()) {
            if (ff == null) {
                return fromIdealSim(fb);
            } else {
                return fromRealisticSim(fb, ff);
            }
        }
        class ArmFX extends SubsystemBase implements ArmMotor {
            TalonFX motor = new TalonFX(canID);
            double targetAngle = 0;
            public ArmFX() {
                config.accept(motor);
            }
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Rotations", motor.getPosition().getValueAsDouble());
                HoundLog.log(name + "/Velocity", motor.getVelocity().getValueAsDouble());
                HoundLog.log(name + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
                HoundLog.log(name + "/Stator Current", motor.getStatorCurrent().getValueAsDouble());
                HoundLog.log(name + "/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
                HoundLog.log(name + "/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
            }
            @Override
            public void setRotations(double target) {
                targetAngle = target;
            }
            @Override
            public void resetRotations(double actual) {
                motor.setPosition(actual);
            }
            @Override
            public double getRotations() {
                return motor.getPosition().getValueAsDouble();
            }
            @Override
            public void periodic() {
                double fbVolts = fb.calculate(motor.getPosition().getValueAsDouble(), targetAngle);
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.ks * Math.signum(fbVolts) 
                        + ff.kg * Math.cos(
                            motor.getPosition().getValueAsDouble() * Math.PI * 2
                        );
                }
                double totalVolts = fbVolts + feedforwardVolts;
                motor.setVoltage(totalVolts);
            }
        }
        return new ArmFX();
    }

    public static ArmMotor fromSparkMax(
        int canID,
        boolean brushed,
        Consumer<CANSparkMax> config,
        FeedbackController fb,
        ArmFeedforward ff
    ) {
        if (RobotBase.isSimulation()) {
            if (ff == null) {
                return fromIdealSim(fb);
            } else {
                return fromRealisticSim(fb, ff);
            }
        }
        class ArmSparkMax extends SubsystemBase implements ArmMotor {
            CANSparkMax motor = new CANSparkMax(canID, brushed ? MotorType.kBrushed : MotorType.kBrushless);
            double targetAngle = 0;
            public ArmSparkMax() {
                config.accept(motor);
            }
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
                HoundLog.log(name + "/Temperature", motor.getMotorTemperature());
                HoundLog.log(name + "/Stator Current", motor.getOutputCurrent());
                HoundLog.log(name + "/Rotations", motor.getEncoder().getPosition());
                HoundLog.log(name + "/Velocity", motor.getEncoder().getVelocity());
            }
            @Override
            public void setRotations(double target) {
                targetAngle = target;
            }
            @Override
            public void resetRotations(double actual) {
                motor.getEncoder().setPosition(actual);
            }
            @Override
            public double getRotations() {
                return motor.getEncoder().getPosition();
            }
            @Override
            public void periodic() {
                double fbVolts = fb.calculate(motor.getEncoder().getPosition(), targetAngle);
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.ks * Math.signum(fbVolts) 
                        + ff.kg * Math.cos(
                            motor.getEncoder().getPosition() * Math.PI * 2
                        );
                }
                double totalVolts = fbVolts + feedforwardVolts;
                motor.setVoltage(totalVolts);
            }
        }
        return new ArmSparkMax();
    }

    public static ArmMotor fromRealisticSim(
        FeedbackController fb,
        ArmFeedforward ff
    ) {
        class ArmRealistic extends SubsystemBase implements ArmMotor {
            FeedforwardSim sim = FeedforwardSim.createArm(ff.kg, ff.ks, ff.kv, ff.ka, new State());
            double targetPos = 0;
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Voltage", sim.getVoltage());
                HoundLog.log(name + "/Velocity", sim.getVelocity());
                HoundLog.log(name + "/Rotations", sim.getPosition());
                HoundLog.log(name + "/Setpoint", fb.getGoal());
            }
            @Override
            public void setRotations(double target) {
                targetPos = target;
            }
            @Override
            public void resetRotations(double actual) {
                sim.resetPosition(actual);
            }
            @Override
            public double getRotations() {
                return sim.getPosition();
            }
            @Override
            public void periodic() {
                if (DriverStation.isDisabled()) {
                    sim.setVoltage(0);
                    return;
                }
                double feedbackVolts = fb.calculate(sim.getPosition(), targetPos);
                double feedforwardVolts = ff.ks * Math.signum(feedbackVolts) 
                        + ff.kg * Math.cos(
                            sim.getPosition() * Math.PI * 2
                        );
                double totalVolts = feedbackVolts + feedforwardVolts;
                sim.setVoltage(totalVolts);
            }
        }
        return new ArmRealistic();
    }

    public static ArmMotor fromIdealSim(
        FeedbackController fb
    ) {
        class ArmIdeal extends SubsystemBase implements ArmMotor {
            State current = new State();
            State target = new State();
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Rotations", current.position);
                HoundLog.log(name + "/Velocity", current.velocity);
                HoundLog.log(name + "/Setpoint", target.position);
            }
            @Override
            public void setRotations(double target) {
                this.target.position = target;
            }
            @Override
            public void resetRotations(double actual) {
                current.position = actual;
            }
            @Override
            public double getRotations() {
                return current.position;
            }
            @Override
            public void periodic() {
                if (DriverStation.isEnabled()) {
                    fb.calculate(current.position, target.position);
                    current = fb.getSetpoint();
                }
            }
        }
        return new ArmIdeal();
    }
}
