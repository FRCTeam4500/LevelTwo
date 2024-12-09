package frc.robot.hardware.motors;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

import static edu.wpi.first.units.Units.*;

public class PositionMotor extends SubsystemBase implements Loggable {
    private double target;
    private double lastVoltage;
    private DoubleConsumer positionSetter;
    private DoubleConsumer voltageSetter;
    private DoubleSupplier positionGetter;
    private DoubleSupplier velocityGetter;
    private FeedbackController fb;
    private ElevatorFeedforward ff;
    private Loggable motorInfo;

    public PositionMotor(
        DoubleConsumer positionSetter,
        DoubleConsumer voltageSetter,
        DoubleSupplier positionGetter,
        DoubleSupplier velocityGetter,
        FeedbackController fb,
        ElevatorFeedforward ff,
        Loggable motorInfo
    ) {
        target = Double.MAX_VALUE;
        this.positionSetter = positionSetter;
        this.voltageSetter = volts -> {
            lastVoltage = volts;
            voltageSetter.accept(volts);
        };
        this.positionGetter = positionGetter;
        this.velocityGetter = velocityGetter;
        this.fb = fb;
        this.ff = ff;
        this.motorInfo = motorInfo;
    }

    public void setTarget(double position) {
        target = position;
    }

    public void resetPosition(double position) {
        positionSetter.accept(position);
    }

    public void coast() {
        target = Double.MAX_VALUE;
    }

    public double getPosition() {
        return positionGetter.getAsDouble();
    }

    public double getVelocity() {
        return velocityGetter.getAsDouble();
    }

    public boolean atTarget() {
        if (target == Double.MAX_VALUE) {
            return true;
        }
        fb.calculate(positionGetter.getAsDouble(), target);
        return fb.atGoal();
    }

    @Override
    public void log(String name) {
        motorInfo.log(name + "/Motor Info");
        HoundLog.log(name + "/Coasting", target == Double.MAX_VALUE);
        HoundLog.log(name + "/Current Position", getPosition());
        HoundLog.log(name + "/Current Velocity", getVelocity());
        if (target != Double.MAX_VALUE) {
            HoundLog.log(name + "/Target Position", target);
        }
        HoundLog.log(name + "/At Target", atTarget());
    }

    @Override
    public void periodic() {
        if (CommandScheduler.getInstance().requiring(this) != null) {
            return;
        }
        if (target == Double.MAX_VALUE || DriverStation.isDisabled()) {
            voltageSetter.accept(0);
            return;
        }
        double fbVolts = fb.calculate(positionGetter.getAsDouble(), target);
        double ffVolts = 0;
        if (ff != null) {
            ffVolts = ff.kg + ff.ks * Math.signum(fbVolts);
        }
        voltageSetter.accept(fbVolts + ffVolts);
    }

    public SysIDCommands getSysID(
        String mechName,
        double voltageRampRate,
        double stepVoltage,
        double timeout
    ) {
        Config config = new Config(
            Volts.of(voltageRampRate).per(Seconds), 
            Volts.of(stepVoltage), 
            Seconds.of(timeout)
        );
        Mechanism mech = new Mechanism(
            voltage -> voltageSetter.accept(voltage.in(Volts)), 
            log -> log.motor("Motor")
                .value("Position", getPosition(), "IDK")
                .value("Velocity", getVelocity(), "IDK")
                .value("Voltage", lastVoltage, "Volts"),
            this,
            mechName
        );
        SysIdRoutine routine = new SysIdRoutine(config, mech);
        return new SysIDCommands(
            routine.dynamic(Direction.kForward), 
            routine.dynamic(Direction.kReverse), 
            routine.quasistatic(Direction.kForward), 
            routine.quasistatic(Direction.kReverse)
        );
    }

    public SysIDCommands getSynchronizedSysID(
        String mechName,
        double voltageRampRate,
        double stepVoltage,
        double timeout,
        PositionMotor... otherMotors
    ) {
        Config config = new Config(
            Volts.of(voltageRampRate).per(Seconds), 
            Volts.of(stepVoltage), 
            Seconds.of(timeout)
        );
        Mechanism mech = new Mechanism(
            voltage -> {
                voltageSetter.accept(voltage.in(Volts));
                for (PositionMotor motor : otherMotors) {
                    motor.voltageSetter.accept(voltage.in(Volts));
                }
            }, 
            log -> {
                log.motor("Motor0")
                    .value("Position", getPosition(), "IDK")
                    .value("Velocity", getVelocity(), "IDK")
                    .value("Voltage", lastVoltage, "Volts");
                for (int i = 0; i < otherMotors.length; i++) {
                    PositionMotor motor = otherMotors[i];
                    log.motor("Motor" + (i + 1))
                        .value("Position", motor.getPosition(), "IDK")
                        .value("Velocity", motor.getVelocity(), "IDK")
                        .value("Voltage", motor.lastVoltage, "Volts");
                }
            },
            this,
            mechName
        );
        SysIdRoutine routine = new SysIdRoutine(config, mech);
        SysIDCommands commands = new SysIDCommands(
            routine.dynamic(Direction.kForward), 
            routine.dynamic(Direction.kReverse), 
            routine.quasistatic(Direction.kForward), 
            routine.quasistatic(Direction.kReverse)
        );
        commands.dynamicForward().addRequirements(otherMotors);
        commands.dynamicReverse().addRequirements(otherMotors);
        commands.quasistaticForward().addRequirements(otherMotors);
        commands.quasistaticReverse().addRequirements(otherMotors);
        return commands;
    }

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
        return new PositionMotor(
            motor::setPosition, 
            motor::setVoltage, 
            () -> motor.getPosition().getValueAsDouble(),
            () -> motor.getVelocity().getValueAsDouble(),
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
                HoundLog.log(name + "/Stator Current", motor.getStatorCurrent().getValueAsDouble());
                HoundLog.log(name + "/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
                HoundLog.log(name + "/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
            }
        );
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
        return new PositionMotor(
            position -> motor.getEncoder().setPosition(position), 
            motor::setVoltage, 
            () -> motor.getEncoder().getPosition(), 
            () -> motor.getEncoder().getVelocity(),
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Applied Volts", motor.getAppliedOutput() * motor.getBusVoltage());
                HoundLog.log(name + "/Temperature", motor.getMotorTemperature());
                HoundLog.log(name + "/Stator Current", motor.getOutputCurrent());
            }
        );
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
        return new PositionMotor(
            position -> motor.setSelectedSensorPosition(position / conversionFactor), 
            voltage -> motor.set(ControlMode.PercentOutput, voltage / motor.getBusVoltage()), 
            () -> motor.getSelectedSensorPosition() * conversionFactor, 
            () -> motor.getSelectedSensorVelocity() * 10 * conversionFactor,
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Bus Voltage", motor.getBusVoltage());
            }
        );
    }

    public static PositionMotor fromRealisticSim(
        FeedbackController fb,
        ElevatorFeedforward ff
    ) {
        FeedforwardSim sim = FeedforwardSim.createElevator(ff.kg, ff.ks, ff.kv, ff.ka, new State());
        return new PositionMotor(
            sim::resetPosition, 
            sim::setVoltage, 
            sim::getPosition, 
            sim::getVelocity,
            fb, 
            ff, 
            name -> {
                HoundLog.log(name + "/Voltage", sim.getVoltage());
            }
        );
    }

    public static PositionMotor fromIdealSim(
        FeedbackController fb
    ) {
        State currentState = new State();
        return new PositionMotor(
            position -> currentState.position = position, 
            voltage -> {
                State nextState = fb.getSetpoint();
                currentState.position = nextState.position;
                currentState.velocity = nextState.velocity;
            }, 
            () -> currentState.position, 
            () -> currentState.velocity,
            fb, 
            null, 
            name -> {
            }
        );
    }
}