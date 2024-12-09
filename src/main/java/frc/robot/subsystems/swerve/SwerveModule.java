package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.motors.PositionMotor;
import frc.robot.hardware.motors.VelocityMotor;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class SwerveModule implements Loggable {
    private VelocityMotor drive;
    private PositionMotor angle;
    private SwerveModuleState targetState;
    public SwerveModule(VelocityMotor drive, PositionMotor angle) {
        this.drive = drive;
        this.angle = angle; 
        targetState = new SwerveModuleState(
            drive.getVelocity(), Rotation2d.fromRotations(MathUtil.inputModulus(angle.getPosition(), 0, 1)));
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, Rotation2d.fromRotations(MathUtil.inputModulus(angle.getPosition(), 0, 1)));
    };

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            drive.getVelocity(), Rotation2d.fromRotations(MathUtil.inputModulus(angle.getPosition(), 0, 1))
        );
    };

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
            drive.getPosition(), 
            Rotation2d.fromRotations(MathUtil.inputModulus(angle.getPosition(), 0, 1))
        );
    }

    public void periodic() {
        drive.setTarget(targetState.speedMetersPerSecond);
        angle.setTarget(targetState.angle.getRotations());
    };

    @Override
    public void log(String name) {
        HoundLog.log(name + "/Current State", getCurrentState());
        HoundLog.log(name + "/Target State", targetState);
        drive.log(name + "/Drive Motor");
        angle.log(name + "/Angle Motor");
    }
}
