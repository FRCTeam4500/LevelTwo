package frc.robot.hardware.motors;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;


import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public interface PositionMotor extends Loggable {
    public void setPosition(double target);
    public void resetPosition(double actual);
    public double getPosition();

    public static PositionMotor fromTalonFX(
        int canID,
        Consumer<TalonFX> config,
        FeedbackController fb,
        ElevatorFeedforward ff
    ) {
        if (RobotBase.isSimulation()) {
            if (ff == null) {
                return fromIdealSim(fb);
            } else {
                return fromRealisticSim(fb, ff);
            }
        }
        class PositionFX extends SubsystemBase implements PositionMotor {
            TalonFX motor = new TalonFX(canID);
            double targetPos = 0;
            public PositionFX() {
                config.accept(motor);
            }
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Position", motor.getPosition().getValueAsDouble());
                HoundLog.log(name + "/Velocity", motor.getVelocity().getValueAsDouble());
                HoundLog.log(name + "/Target", targetPos);
                HoundLog.log(name + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
                HoundLog.log(name + "/Stator Current", motor.getStatorCurrent().getValueAsDouble());
                HoundLog.log(name + "/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
                HoundLog.log(name + "/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
            }
            @Override
            public void setPosition(double target) {
                targetPos = target;
            }
            @Override
            public void resetPosition(double actual) {
                motor.setPosition(actual);
            }
            @Override
            public double getPosition() {
                return motor.getPosition().getValueAsDouble();
            }
            @Override
            public void periodic() {
                double fbVolts = fb.calculate(motor.getPosition().getValueAsDouble(), targetPos);
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.kg + ff.ks * Math.signum(fbVolts);
                }
                double totalVolts = fbVolts + feedforwardVolts;
                motor.setVoltage(totalVolts);
            }
        }
        return new PositionFX();
    }

    public static PositionMotor fromSparkMax(
        int canID,
        boolean brushed,
        Consumer<CANSparkMax> config,
        FeedbackController fb,
        ElevatorFeedforward ff
    ) {
        if (RobotBase.isSimulation()) {
            if (ff == null) {
                return fromIdealSim(fb);
            } else {
                return fromRealisticSim(fb, ff);
            }
        }
        class PositionSparkMax extends SubsystemBase implements PositionMotor {
            CANSparkMax motor = new CANSparkMax(canID, brushed ? MotorType.kBrushed : MotorType.kBrushless);
            double targetPos = 0;
            public PositionSparkMax() {
                config.accept(motor);
            }
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
                HoundLog.log(name + "/Temperature", motor.getMotorTemperature());
                HoundLog.log(name + "/Stator Current", motor.getOutputCurrent());
                HoundLog.log(name + "/Position", motor.getEncoder().getPosition());
                HoundLog.log(name + "/Velocity", motor.getEncoder().getVelocity());
                HoundLog.log(name + "/Target", targetPos);
            }
            @Override
            public void setPosition(double target) {
                targetPos = target;
            }
            @Override
            public void resetPosition(double actual) {
                motor.getEncoder().setPosition(actual);
            }
            @Override
            public double getPosition() {
                return motor.getEncoder().getPosition();
            }
            @Override
            public void periodic() {
                double fbVolts = fb.calculate(motor.getEncoder().getPosition(), targetPos);
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.kg + ff.ks * Math.signum(fbVolts);
                }
                double totalVolts = fbVolts + feedforwardVolts;
                motor.setVoltage(totalVolts);
            }
        }
        return new PositionSparkMax();
    }

    public static PositionMotor fromTalonSRX(
        int canID,
        double conversionFactor,
        Consumer<TalonSRX> config,
        FeedbackController fb,
        ElevatorFeedforward ff
    ) {
        if (RobotBase.isSimulation()) {
            if (ff == null) {
                return fromIdealSim(fb);
            } else {
                return fromRealisticSim(fb, ff);
            }
        }
        class PositionSRX extends SubsystemBase implements PositionMotor {
            TalonSRX motor = new TalonSRX(canID);
            double targetPos = 0;
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Position", motor.getSelectedSensorPosition() * conversionFactor);
                HoundLog.log(name + "/Velocity", motor.getSelectedSensorVelocity() * 10 * conversionFactor);
                HoundLog.log(name + "/Target", targetPos);
            }

            @Override
            public void setPosition(double target) {
                targetPos = target;
            }

            @Override
            public void resetPosition(double actual) {
                motor.setSelectedSensorPosition(targetPos / conversionFactor);
            }

            @Override
            public double getPosition() {
                return motor.getSelectedSensorPosition() * conversionFactor;
            }

            @Override
            public void periodic() {
                if (DriverStation.isDisabled()) {
                    motor.set(ControlMode.PercentOutput, 0);
                    return;
                }
                double feedbackVolts = fb.calculate(motor.getSelectedSensorPosition() * conversionFactor, targetPos);
                double feedforwardVolts = ff.kg + ff.ks * Math.signum(feedbackVolts);
                double totalVolts = feedbackVolts + feedforwardVolts;
                if (DriverStation.isDisabled()) {
                    totalVolts = 0;
                }
                motor.set(ControlMode.PercentOutput, totalVolts / motor.getBusVoltage());
            }
        }
        return new PositionSRX();
    }

    public static PositionMotor fromRealisticSim(
        FeedbackController fb,
        ElevatorFeedforward ff
    ) {
        class PositionRealistic extends SubsystemBase implements PositionMotor {
            FeedforwardSim sim = FeedforwardSim.createElevator(ff.kg, ff.ks, ff.kv, ff.ka, new State());
            double targetPos = 0;
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Voltage", sim.getVoltage());
                HoundLog.log(name + "/Velocity", sim.getVelocity());
                HoundLog.log(name + "/Position", sim.getPosition());
                HoundLog.log(name + "/Setpoint", fb.getGoal());
            }
            @Override
            public void setPosition(double target) {
                targetPos = target;
            }
            @Override
            public void resetPosition(double actual) {
                sim.resetPosition(actual);
            }
            @Override
            public double getPosition() {
                return sim.getPosition();
            }
            @Override
            public void periodic() {
                if (DriverStation.isDisabled()) {
                    sim.setVoltage(0);
                    return;
                }
                double feedbackVolts = fb.calculate(sim.getPosition(), targetPos);
                double feedforwardVolts = ff.kg + ff.ks * Math.signum(feedbackVolts);
                double totalVolts = feedbackVolts + feedforwardVolts;
                if (DriverStation.isDisabled()) {
                    totalVolts = 0;
                }
                sim.setVoltage(totalVolts);
            }
        }
        return new PositionRealistic();
    }

    public static PositionMotor fromIdealSim(
        FeedbackController fb
    ) {
        class PositionIdeal extends SubsystemBase implements PositionMotor {
            State current = new State();
            State target = new State();
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Position", current.position);
                HoundLog.log(name + "/Velocity", current.velocity);
                HoundLog.log(name + "/Setpoint", target.position);
            }
            @Override
            public void setPosition(double target) {
                this.target.position = target;
            }
            @Override
            public void resetPosition(double actual) {
                current.position = actual;
            }
            @Override
            public double getPosition() {
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
        return new PositionIdeal();
    }
}
