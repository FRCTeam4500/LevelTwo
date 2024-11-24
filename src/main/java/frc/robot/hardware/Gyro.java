package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

import java.util.function.DoubleSupplier;

public interface Gyro extends Loggable {
    public Rotation2d getAngle();

    public static Gyro fromNavX() {
        AHRS navx = new AHRS();
        return new Gyro() {
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Connected", navx.isConnected());
                HoundLog.log(name + "/Pitch", navx.getPitch());
                HoundLog.log(name + "/Roll", navx.getRoll());
                HoundLog.log(name + "/Angle", navx.getYaw());
            }

            @Override
            public Rotation2d getAngle() {
                return navx.getRotation2d();
            }
        };
    }

    public static Gyro fromSim(DoubleSupplier radiansPerSecond) {
        class GyroSim extends SubsystemBase implements Gyro {
            double angle = 0;
            @Override
            public void log(String name) {
                HoundLog.log(name + "/Angle", Math.toDegrees(angle));    
            }

            @Override
            public Rotation2d getAngle() {
                return Rotation2d.fromRadians(angle);
            }

            @Override
            public void periodic() {
                angle += radiansPerSecond.getAsDouble() * 0.02;
            }
        }
        return new GyroSim();
    }
}
