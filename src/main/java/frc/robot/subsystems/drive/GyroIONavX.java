package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.SerialPort;
import com.kauailabs.navx.frc.AHRS;

public class GyroIONavX implements GyroIO {
    
    private final AHRS gyro;

    public GyroIONavX() {
        gyro = new AHRS(SerialPort.Port.kMXP);

        if (gyro != null) {
            gyro.calibrate();
        }
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        if (gyro != null) {
            inputs.gyroConnected = gyro.isConnected();
            inputs.gyroYawPositionRad = Math.toRadians(-gyro.getAngle());
            inputs.gyroYawVelocityRadPerSec = Math.toRadians(gyro.getRate());
            inputs.gyroPitchPositionRad = Math.toRadians(gyro.getRoll());
            inputs.gyroRollPositionRad = Math.toRadians(gyro.getPitch());
            inputs.gyroZAccelMetersPerSec2 = gyro.getWorldLinearAccelZ() * 9.806;
        } else {
            inputs.gyroConnected = false;
        }
    }
    
}