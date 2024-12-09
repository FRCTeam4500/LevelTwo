package frc.robot.utilities;

import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public interface FeedbackController {
    public double calculate(double measurement, double goal);
    public double getGoal();
    public State getSetpoint();
    public boolean atGoal();

    public static FeedbackController fromPID(
        PIDController pid,
        Consumer<PIDController> config
    ) {
        config.accept(pid);
        return new FeedbackController() {
            @Override
            public double calculate(double measurement, double goal) {
                return pid.calculate(measurement, goal);
            }

            @Override
            public double getGoal() {
                return pid.getSetpoint();
            }

            @Override
            public State getSetpoint() {
                return new State(pid.getSetpoint(), 0);
            }

            @Override
            public boolean atGoal() {
                return pid.atSetpoint();
            }
        };
    }

    public static FeedbackController fromProfiledPID(
        ProfiledPIDController pid,
        Consumer<ProfiledPIDController> config
    ) {
        config.accept(pid);
        return new FeedbackController() {
            @Override
            public double calculate(double measurement, double goal) {
                return pid.calculate(measurement, goal);
            }

            @Override
            public double getGoal() {
                return pid.getGoal().position;
            }

            @Override
            public State getSetpoint() {
                return pid.getSetpoint();
            }

            @Override
            public boolean atGoal() {
                return pid.atSetpoint();
            }
        };
    }
}
