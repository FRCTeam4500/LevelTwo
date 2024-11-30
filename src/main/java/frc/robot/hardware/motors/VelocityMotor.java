package frc.robot.hardware.motors;

import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public interface VelocityMotor extends Loggable {
    public void gotoVelocity(double target);
    public void resetPosition(double position);
    public double getPosition();
    public double getVelocity();
    public boolean atTarget();

    public static VelocityMotor fromTalonFX(
        int canID,
        Consumer<TalonFX> config,
        FeedbackController fb,
        SimpleMotorFeedforward ff
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

        return new VelocityMotor() {
            double targetVel = 0;
            @Override
            public void log(String name) {
                fb.calculate(getVelocity(), targetVel);
                HoundLog.log(name + "/Position", motor.getPosition().getValueAsDouble());
                HoundLog.log(name + "/Velocity", motor.getVelocity().getValueAsDouble());
                HoundLog.log(name + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
                HoundLog.log(name + "/Stator Current", motor.getStatorCurrent().getValueAsDouble());
                HoundLog.log(name + "/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
                HoundLog.log(name + "/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
                HoundLog.log(name + "/At Target", fb.atGoal());
                HoundLog.log(name + "/Target", targetVel);
            }

            @Override
            public void gotoVelocity(double target) {
                targetVel = target;
                double feedbackVolts =  fb.calculate(
                    motor.getVelocity().getValueAsDouble(), targetVel
                );
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.calculate(targetVel);
                }
                double volts = feedbackVolts + feedforwardVolts;
                motor.setVoltage(volts);
            }

            @Override
            public void resetPosition(double position) {
                motor.setPosition(position);
            }

            @Override
            public double getPosition() {
                return motor.getPosition().getValueAsDouble();
            }

            @Override
            public double getVelocity() {
                return motor.getVelocity().getValueAsDouble();
            }

            @Override
            public boolean atTarget() {
                fb.calculate(getVelocity(), targetVel);
                return fb.atGoal();
            }
        };
    }

    public static VelocityMotor fromSparkmax(
        int canID,
        boolean brushed,
        Consumer<CANSparkMax> config,
        FeedbackController fb,
        SimpleMotorFeedforward ff
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
        return new VelocityMotor() {
            double targetVel = 0;

            @Override
            public void log(String name) {
                fb.calculate(getVelocity(), targetVel);
                HoundLog.log(name + "/Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
                HoundLog.log(name + "/Temperature", motor.getMotorTemperature());
                HoundLog.log(name + "/Stator Current", motor.getOutputCurrent());
                HoundLog.log(name + "/Position", motor.getEncoder().getPosition());
                HoundLog.log(name + "/Velocity", motor.getEncoder().getVelocity());
                HoundLog.log(name + "/At Target", fb.atGoal());
                HoundLog.log(name + "/Target", targetVel);
            }

            @Override
            public void gotoVelocity(double target) {
                targetVel = target;
                double feedbackVolts =  fb.calculate(
                    motor.getEncoder().getVelocity(), targetVel
                );
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.calculate(targetVel);
                }
                double volts = feedbackVolts + feedforwardVolts;
                motor.setVoltage(volts);
            }

            @Override
            public void resetPosition(double position) {
                motor.getEncoder().setPosition(position);
            }

            @Override
            public double getPosition() {
                return motor.getEncoder().getPosition();
            }

            @Override
            public double getVelocity() {
                return motor.getEncoder().getVelocity();
            }

            @Override
            public boolean atTarget() {
                fb.calculate(getVelocity(), targetVel);
                return fb.atGoal();
            }
        };
    }

    public static VelocityMotor fromRealisticSim(
        FeedbackController fb,
        SimpleMotorFeedforward ff
    ) {
        FeedforwardSim sim = FeedforwardSim.createFlywheel(ff.ks, ff.kv, ff.ka, new State());
        return new VelocityMotor() {
            double targetVel = 0;
            @Override
            public void log(String name) {
                fb.calculate(getVelocity(), targetVel);
                HoundLog.log(name + "/Voltage", sim.getVoltage());
                HoundLog.log(name + "/Position", sim.getPosition());
                HoundLog.log(name + "/Velocity", sim.getVelocity());
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoVelocity(double target) {
                targetVel = target;
                double fbVolts =  fb.calculate(
                    sim.getVelocity(), targetVel
                );
                double ffVolts = ff.calculate(targetVel);
                double volts = fbVolts + ffVolts;
                sim.setVoltage(volts);
            }
            @Override
            public void resetPosition(double position) {
                sim.resetPosition(position);
            }
            @Override
            public double getPosition() {
                return sim.getPosition();
            }
            @Override
            public double getVelocity() {
                return sim.getVelocity();
            }
            @Override
            public boolean atTarget() {
                fb.calculate(getVelocity(), targetVel);
                return fb.atGoal();
            }
        };
    }

    public static VelocityMotor fromIdealSim(
        FeedbackController fb
    ) {
        return new VelocityMotor() {
            State currentState = new State();
            State targetState = new State();
            double position = 0;
            @Override
            public void log(String name) {
                fb.calculate(getVelocity(), targetState.position);
                HoundLog.log(name + "/Position", position);
                HoundLog.log(name + "/Velocity", currentState.position);
                HoundLog.log(name + "/At Target", fb.atGoal());
            }
            @Override
            public void gotoVelocity(double target) {
                this.targetState.position = target;
                position += currentState.position * 0.02;
                fb.calculate(currentState.position, targetState.position);
                currentState = fb.getSetpoint();
            }
            @Override
            public void resetPosition(double position) {
                this.position = position;
            }
            @Override
            public double getPosition() {
                return position;
            }
            @Override
            public double getVelocity() {
                return currentState.position;
            }
            @Override
            public boolean atTarget() {
                fb.calculate(getVelocity(), targetState.position);
                return fb.atGoal();
            }
        };
    }
}