package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import static edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class DriveIOSim implements DriveIO {
    private final double wheelRadius = Units.inchesToMeters(3.0);
    private final DifferentialDrivetrainSim driveSim = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDualCIMPerSide,
            KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);
    private final GyroIO gyroIO = new GyroIOSim();

    private double leftVolts = 0.0, rightVolts = 0.0;

    public class GyroIOSim implements GyroIO {
        @Override
        public void updateInputs(GyroIOInputs inputs) {
            inputs.gyroYawPositionRad = driveSim.getHeading().getRadians();
        }
    }

    public GyroIO getGyroIO() {
        return gyroIO;
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            leftVolts = 0;
            rightVolts = 0;
        }
        
        driveSim.setInputs(leftVolts, rightVolts);

        driveSim.update(0.02);

        inputs.leftPositionRad = driveSim.getLeftPositionMeters() / wheelRadius;
        inputs.leftVelocityRadPerSec = driveSim.getLeftVelocityMetersPerSecond() / wheelRadius;
        inputs.leftCurrentAmps = new double[] {driveSim.getLeftCurrentDrawAmps()};
        inputs.leftAppliedVolts = leftVolts;

        inputs.rightPositionRad = driveSim.getRightPositionMeters() / wheelRadius;
        inputs.rightVelocityRadPerSec = driveSim.getRightVelocityMetersPerSecond() / wheelRadius;
        inputs.rightCurrentAmps = new double[] {driveSim.getRightCurrentDrawAmps()};
        inputs.rightAppliedVolts = rightVolts;
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
        rightVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
    }
}