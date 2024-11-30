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
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public interface PositionMotor extends Loggable {
    public void gotoPosition(double target);
    public void resetPosition(double actual);
    public double getPosition();
    public boolean atTarget();

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
        TalonFX motor = new TalonFX(canID);
        config.accept(motor);
        return new PositionMotor() {
            double targetPos = 0;
            @Override
            public void log(String name) {
                fb.calculate(getPosition(), targetPos);
                HoundLog.log(name + "/Position", motor.getPosition().getValueAsDouble());
                HoundLog.log(name + "/Velocity", motor.getVelocity().getValueAsDouble());
                HoundLog.log(name + "/Target", targetPos);
                HoundLog.log(name + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
                HoundLog.log(name + "/Stator Current", motor.getStatorCurrent().getValueAsDouble());
                HoundLog.log(name + "/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
                HoundLog.log(name + "/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoPosition(double target) {
                targetPos = target;
                double fbVolts = fb.calculate(motor.getPosition().getValueAsDouble(), targetPos);
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.kg + ff.ks * Math.signum(fbVolts);
                }
                double totalVolts = fbVolts + feedforwardVolts;
                motor.setVoltage(totalVolts);
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
            public boolean atTarget() {
                return fb.atGoal();
            }
        };
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
        CANSparkMax motor = new CANSparkMax(canID, brushed ? MotorType.kBrushed : MotorType.kBrushless); 
        config.accept(motor);
        return new PositionMotor() {
            double targetPos = 0;
            @Override
            public void log(String name) {
                fb.calculate(getPosition(), targetPos);
                HoundLog.log(name + "/Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
                HoundLog.log(name + "/Temperature", motor.getMotorTemperature());
                HoundLog.log(name + "/Stator Current", motor.getOutputCurrent());
                HoundLog.log(name + "/Position", motor.getEncoder().getPosition());
                HoundLog.log(name + "/Velocity", motor.getEncoder().getVelocity());
                HoundLog.log(name + "/Target", targetPos);
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoPosition(double target) {
                targetPos = target;
                double fbVolts = fb.calculate(motor.getEncoder().getPosition(), targetPos);
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.kg + ff.ks * Math.signum(fbVolts);
                }
                double totalVolts = fbVolts + feedforwardVolts;
                motor.setVoltage(totalVolts);
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
            public boolean atTarget() {
                return fb.atGoal();
            }
        };
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
        TalonSRX motor = new TalonSRX(canID);
        config.accept(motor);
        return new PositionMotor() {
            double targetPos = 0;
            @Override
            public void log(String name) {
                fb.calculate(getPosition(), targetPos);
                HoundLog.log(name + "/Position", motor.getSelectedSensorPosition() * conversionFactor);
                HoundLog.log(name + "/Velocity", motor.getSelectedSensorVelocity() * 10 * conversionFactor);
                HoundLog.log(name + "/Target", targetPos);
                HoundLog.log(name + "/At Target", fb.atGoal());
                HoundLog.log(name + "/Bus Voltage", motor.getBusVoltage());
            }
            @Override
            public void gotoPosition(double target) {
                targetPos = target;
                double feedbackVolts = fb.calculate(motor.getSelectedSensorPosition() * conversionFactor, targetPos);
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.kg + ff.ks * Math.signum(feedbackVolts);
                }
                double totalVolts = feedbackVolts + feedforwardVolts;
                if (DriverStation.isDisabled()) {
                    totalVolts = 0;
                }
                motor.set(ControlMode.PercentOutput, totalVolts / motor.getBusVoltage());
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
            public boolean atTarget() {
                return fb.atGoal();
            }
        };
    }

    public static PositionMotor fromRealisticSim(
        FeedbackController fb,
        ElevatorFeedforward ff
    ) {
        FeedforwardSim sim = FeedforwardSim.createElevator(ff.kg, ff.ks, ff.kv, ff.ka, new State());
        return new PositionMotor() {
            double targetPos = 0;
            @Override
            public void log(String name) {
                fb.calculate(getPosition(), targetPos);
                HoundLog.log(name + "/Voltage", sim.getVoltage());
                HoundLog.log(name + "/Velocity", sim.getVelocity());
                HoundLog.log(name + "/Position", sim.getPosition());
                HoundLog.log(name + "/Setpoint", fb.getGoal());
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoPosition(double target) {
                targetPos = target;
                double feedbackVolts = fb.calculate(sim.getPosition(), targetPos);
                double feedforwardVolts = ff.kg + ff.ks * Math.signum(feedbackVolts);
                double totalVolts = feedbackVolts + feedforwardVolts;
                if (DriverStation.isDisabled()) {
                    totalVolts = 0;
                }
                sim.setVoltage(totalVolts);
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
            public boolean atTarget() {
                return fb.atGoal();
            }
        };
    }

    public static PositionMotor fromIdealSim(
        FeedbackController fb
    ) {
        return new PositionMotor() {
            State currentState = new State();
            State targetState = new State();
            @Override
            public void log(String name) {
                fb.calculate(getPosition(), targetState.position);
                HoundLog.log(name + "/Position", currentState.position);
                HoundLog.log(name + "/Velocity", currentState.velocity);
                HoundLog.log(name + "/Setpoint", targetState.position);
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoPosition(double target) {
                this.targetState.position = target;
                fb.calculate(currentState.position, targetState.position);
                currentState = fb.getSetpoint();
            }
            @Override
            public void resetPosition(double actual) {
                currentState.position = actual;
            }
            @Override
            public double getPosition() {
                return currentState.position;
            }
            @Override
            public boolean atTarget() {
                fb.calculate(getPosition(), targetState.position);
                return fb.atGoal();
            }
        };
    }
}
