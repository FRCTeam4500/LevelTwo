package frc.robot.utilities;

import java.util.function.BiConsumer;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeedforwardSim extends SubsystemBase {
    private BiConsumer<State, Double> calc;
    private State state;
    private double volts;

    /**
     * @param calc A function that takes the current state and the voltage, and
     * modifies the passed in state to be the state after 0.02 seconds
     * @param initalState The inital state of the mechanism
     */
    public FeedforwardSim(BiConsumer<State, Double> calc, State initalState) {
        this.calc = calc;
        this.state = initalState;
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            volts = 0;
        }
        calc.accept(state, volts);
    }

    public void setVoltage(double volts) {
        this.volts = volts;
    }

    public double getVoltage() {
        return volts;
    }

    public double getPosition() {
        return state.position;
    }

    public double getVelocity() {
        return state.velocity;
    }

    public void resetPosition(double newPosition) {
        state.position = newPosition;
    }

    /**
     * Creates a feedforward sim model for a flywheel mechanism. 
     * This is also applicable for systems that extend instead of rotating,
     * as long as they aren't affected by gravity. For example, a horizontal elevator.
     * <p>
     * The feedforward constants should be obtained via SysId
     * @param kS The voltage needed to overcome the friction forces in the system.
     * @param kV The voltage needed to cause a given constant velocity.
     * @param kA The voltage needed to cause a given acceleration
     * @param initialState The inital position and velocity of the mechanism
     */
    public static FeedforwardSim createFlywheel(double kS, double kV, double kA, State initalState) {
        return new FeedforwardSim(
            (state, voltage) -> {
                double staticVolts = Math.signum(state.velocity) * kS;
                double velocityVolts = state.velocity * kV;
                double deltaVel = 0.02 * (voltage - staticVolts - velocityVolts) / kA;
                double averageVel = state.velocity + deltaVel / 2;
                state.position += 0.02 * averageVel;
                state.velocity += deltaVel;
                
            }, initalState
        );
    }

    /**
     * Creates a feedforward sim model for a elevator mechanism. 
     * This is applicable for systems that extends against a constant force of gravity.
     * <p>
     * The feedforward constants should be obtained via SysId
     * @param kG The voltage need to overcome the gravitational force on the system.
     * @param kS The voltage needed to overcome the friction forces in the system.
     * @param kV The voltage needed to cause a given constant velocity.
     * @param kA The voltage needed to cause a given acceleration.
     * @param initialState The inital position and velocity of the mechanism.
     */
    public static FeedforwardSim createElevator(double kG, double kS, double kV, double kA, State initialState) {
        return new FeedforwardSim(
            (state, volts) -> {
                double staticVolts = Math.signum(state.velocity) * kS;
                double velocityVolts = state.velocity * kV;
                double deltaVel = 0.02 * (volts - kG - staticVolts - velocityVolts) / kA;
                double averageVel = state.velocity + deltaVel / 2;
                state.position += 0.02 * averageVel;
                state.velocity += deltaVel;
            }, initialState
        );
    }

    /**
     * Creates a feedforward sim model for a jointed arm mechanism. 
     * This is applicable for systems that rotate vertically, and face different
     * gravitational forces depending on their angle. Note that 0 rotations
     * must corespond to the arm being parallel to the ground
     * <p>
     * The feedforward constants should be obtained via SysId
     * @param kG The voltage need to overcome the gravitational force on the system
     * when the mechanism is parallel to the ground (0 rotations).
     * @param kS The voltage needed to overcome the friction forces in the system.
     * @param kV The voltage needed to cause a given constant velocity.
     * @param kA The voltage needed to cause a given acceleration.
     * @param initialState The inital position and velocity of the mechanism in rotations and rotations/second.
     */
    public static FeedforwardSim createArm(double kG, double kS, double kV, double kA, State initialState) {
        return new FeedforwardSim(
            (state, volts) -> {
                double gravityVolts = Math.cos(state.position * 2 * Math.PI) * kG;
                double staticVolts = Math.signum(state.velocity) * kS;
                double velocityVolts = state.velocity * kV;
                double deltaVel = 0.02 * (volts - gravityVolts - staticVolts - velocityVolts) / kA;
                double averageVel = state.velocity + deltaVel / 2;
                state.position += 0.02 * averageVel;
                state.velocity += deltaVel;
            }, initialState  
        );
    }
}
