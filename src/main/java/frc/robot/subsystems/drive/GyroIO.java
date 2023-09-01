package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean gyroConnected = false;
        public double gyroYawPositionRad = 0.0;
        public double gyroYawVelocityRadPerSec = 0.0;
        public double gyroPitchPositionRad = 0.0;
        public double gyroRollPositionRad = 0.0;
        public double gyroZAccelMetersPerSec2 = 0.0;
    }

    public void updateInputs(GyroIOInputs inputs);
}