package frc.robot.hardware.motors;

import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public interface VelocityMotor extends Loggable {
    public void setVelocity(double target);
    public void resetPosition(double position);
    public double getPosition();
    public double getVelocity();

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
        class VelocityFX extends SubsystemBase implements VelocityMotor{
            TalonFX motor;
            double targetVel;

            public VelocityFX() {
                motor = new TalonFX(canID);
                config.accept(motor);
                targetVel = 0;
            }

            @Override
            public void log(String name) {
                HoundLog.log(name + "/Position", motor.getPosition().getValueAsDouble());
                HoundLog.log(name + "/Velocity", motor.getVelocity().getValueAsDouble());
                HoundLog.log(name + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
                HoundLog.log(name + "/Stator Current", motor.getStatorCurrent().getValueAsDouble());
                HoundLog.log(name + "/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
                HoundLog.log(name + "/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
            }

            @Override
            public void setVelocity(double target) {
                targetVel = target;
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
            public void periodic() {
                double feedbackVolts =  fb.calculate(
                    motor.getVelocity().getValueAsDouble(), targetVel
                );
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.calculate(targetVel);
                }
                double volts = feedbackVolts + feedforwardVolts;
                if (fb.atGoal()) {
                    motor.setVoltage(feedforwardVolts);
                } else {
                    motor.setVoltage(volts);
                }
            }
        }
        return new VelocityFX();
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
        class VelocitySparkMax extends SubsystemBase implements VelocityMotor{
            CANSparkMax motor;
            double targetVel;

            public VelocitySparkMax() {
                motor = new CANSparkMax(canID, brushed ? MotorType.kBrushed : MotorType.kBrushless);
                config.accept(motor);
                targetVel = 0;
            }

            @Override
            public void log(String name) {
                HoundLog.log(name + "/Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
                HoundLog.log(name + "/Temperature", motor.getMotorTemperature());
                HoundLog.log(name + "/Stator Current", motor.getOutputCurrent());
                HoundLog.log(name + "/Position", motor.getEncoder().getPosition());
                HoundLog.log(name + "/Velocity", motor.getEncoder().getVelocity());
            }

            @Override
            public void setVelocity(double target) {
                targetVel = target;
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
            public void periodic() {
                double feedbackVolts =  fb.calculate(
                    motor.getEncoder().getVelocity(), targetVel
                );
                double feedforwardVolts = 0;
                if (ff != null) {
                    feedforwardVolts = ff.calculate(targetVel);
                }
                double volts = feedbackVolts + feedforwardVolts;
                if (fb.atGoal()) {
                    motor.setVoltage(feedforwardVolts);
                } else {
                    motor.setVoltage(volts);
                }
            }
        }
        return new VelocitySparkMax();
    }

    public static VelocityMotor fromRealisticSim(
        FeedbackController fb,
        SimpleMotorFeedforward ff
    ) {
        class VelocityRealistic extends SubsystemBase implements VelocityMotor {
            FeedforwardSim sim = FeedforwardSim.createFlywheel(ff.ks, ff.kv, ff.ka, new State());
            double targetVel = 0;
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Voltage", sim.getVoltage());
                HoundLog.log(name + "/Position", sim.getPosition());
                HoundLog.log(name + "/Velocity", sim.getVelocity());
            }
            @Override
            public void setVelocity(double target) {
                targetVel = target;
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
            public void periodic() {
                if (DriverStation.isDisabled()) {
                    sim.setVoltage(0);
                    return;
                }
                double fbVolts =  fb.calculate(
                    sim.getVelocity(), targetVel
                );
                double ffVolts = ff.calculate(targetVel);
                double volts = fbVolts + ffVolts;
                if (fb.atGoal()) {
                    sim.setVoltage(ffVolts);
                } else {
                    sim.setVoltage(volts);
                }
            }
        }
        return new VelocityRealistic();
    }

    public static VelocityMotor fromIdealSim(
        FeedbackController fb
    ) {
        class VelocityIdeal extends SubsystemBase implements VelocityMotor {
            State current = new State();
            State target = new State();
            double position = 0;
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Position", position);
                HoundLog.log(name + "/Velocity", current.position);
            }
            @Override
            public void setVelocity(double target) {
                this.target.position = target;
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
                return current.position;
            }
            @Override
            public void periodic() {
                position += current.position * 0.02;
                if (DriverStation.isEnabled()) {
                    fb.calculate(current.position, target.position);
                    current = fb.getSetpoint();
                }
            }
        }
        return new VelocityIdeal();
    }
}
