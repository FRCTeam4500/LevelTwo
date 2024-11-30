package frc.robot.hardware.motors;

import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public interface ArmMotor extends Loggable {
    public void gotoAngle(double targetRotations);
    public void resetAngle(double actualRotations);
    public double getRotations();
    public boolean atTarget();

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
        TalonFX motor = new TalonFX(canID);
        config.accept(motor);
        return new ArmMotor() {
            double targetAngle = 0;
            @Override
            public void log(String name) {
                fb.calculate(getRotations(), targetAngle);
                HoundLog.log(name + "/Rotations", motor.getPosition().getValueAsDouble());
                HoundLog.log(name + "/Velocity", motor.getVelocity().getValueAsDouble());
                HoundLog.log(name + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
                HoundLog.log(name + "/Stator Current", motor.getStatorCurrent().getValueAsDouble());
                HoundLog.log(name + "/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
                HoundLog.log(name + "/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoAngle(double targetRotations) {
                targetAngle = targetRotations;
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
            @Override
            public void resetAngle(double actual) {
                motor.setPosition(actual);
            }
            @Override
            public double getRotations() {
                return motor.getPosition().getValueAsDouble();
            }
            @Override
            public boolean atTarget() {
                fb.calculate(getRotations(), targetAngle);
                return fb.atGoal();
            }
        };
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
        CANSparkMax motor = new CANSparkMax(canID, brushed ? MotorType.kBrushed : MotorType.kBrushless);
        config.accept(motor);
        return new ArmMotor() {
            double targetAngle = 0;
            @Override
            public void log(String name) {
                fb.calculate(getRotations(), targetAngle);
                HoundLog.log(name + "/Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
                HoundLog.log(name + "/Temperature", motor.getMotorTemperature());
                HoundLog.log(name + "/Stator Current", motor.getOutputCurrent());
                HoundLog.log(name + "/Rotations", motor.getEncoder().getPosition());
                HoundLog.log(name + "/Velocity", motor.getEncoder().getVelocity());
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoAngle(double target) {
                targetAngle = target;
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
            @Override
            public void resetAngle(double actual) {
                motor.getEncoder().setPosition(actual);
            }
            @Override
            public double getRotations() {
                return motor.getEncoder().getPosition();
            }
            @Override
            public boolean atTarget() {
                fb.calculate(getRotations(), targetAngle);
                return fb.atGoal();
            }
        };
    }

    public static ArmMotor fromRealisticSim(
        FeedbackController fb,
        ArmFeedforward ff
    ) {
        FeedforwardSim sim = FeedforwardSim.createArm(ff.kg, ff.ks, ff.kv, ff.ka, new State());
        return new ArmMotor() {
            double targetAngle = 0;
            @Override
            public void log(String name) {
                fb.calculate(getRotations(), targetAngle);
                HoundLog.log(name + "/Voltage", sim.getVoltage());
                HoundLog.log(name + "/Velocity", sim.getVelocity());
                HoundLog.log(name + "/Rotations", sim.getPosition());
                HoundLog.log(name + "/Setpoint", fb.getGoal());
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoAngle(double target) {
                targetAngle = target;
                double feedbackVolts = fb.calculate(sim.getPosition(), targetAngle);
                double feedforwardVolts = ff.ks * Math.signum(feedbackVolts) 
                        + ff.kg * Math.cos(
                            sim.getPosition() * Math.PI * 2
                        );
                double totalVolts = feedbackVolts + feedforwardVolts;
                sim.setVoltage(totalVolts);
            }
            @Override
            public void resetAngle(double actual) {
                sim.resetPosition(actual);
            }
            @Override
            public double getRotations() {
                return sim.getPosition();
            }
            @Override
            public boolean atTarget() {
                fb.calculate(getRotations(), targetAngle);
                return fb.atGoal();
            }
        };
    }

    public static ArmMotor fromIdealSim(
        FeedbackController fb
    ) {
        return new ArmMotor() {
            State currentState = new State();
            State targetState = new State();
            @Override
            public void log(String name) {
                fb.calculate(currentState.position, targetState.position);
                HoundLog.log(name + "/Rotations", currentState.position);
                HoundLog.log(name + "/Velocity", currentState.velocity);
                HoundLog.log(name + "/Setpoint", targetState.position);
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoAngle(double target) {
                this.targetState.position = target;
                fb.calculate(currentState.position, targetState.position);
                currentState = fb.getSetpoint();
            }
            @Override
            public void resetAngle(double actual) {
                currentState.position = actual;
            }
            @Override
            public double getRotations() {
                return currentState.position;
            }
            @Override
            public boolean atTarget() {
                fb.calculate(currentState.position, targetState.position);
                return fb.atGoal();
            }
        };
    }
}
