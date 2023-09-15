package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    // constants
    public static double trackwidth = 1.3; // meters (this is made up)
    public static double encoderAfterReduction = 1.0 / ((9.0 / 62.0) * (18.0 / 30.0));
    public static double wheelRadius = Units.inchesToMeters(6.0 / 2);

    private final DriveIO driveIO;
    private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    private double lastLeftDistanceMeters = 0.0; 
    private double lastRightDistanceMeters = 0.0;
    private double lastYawAngleRads = 0.0;
    private double leftDelta = 0.0;
    private double rightDelta = 0.0;

    public Drive(DriveIO driveIO, GyroIO gyroIO) {
        this.driveIO = driveIO;
        this.gyroIO = gyroIO;
    }

    @Override
    public void periodic() {
        lastYawAngleRads = gyroInputs.gyroYawPositionRad;
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        lastLeftDistanceMeters = getLeftDistanceMeters();
        lastRightDistanceMeters = getRightDistanceMeters();
        driveIO.updateInputs(driveInputs);
        Logger.getInstance().processInputs("Drive/Drive", driveInputs);

        Rotation2d yaw = new Rotation2d(gyroInputs.gyroYawPositionRad);
        leftDelta = getLeftDistanceMeters() - lastLeftDistanceMeters;
        rightDelta = getRightDistanceMeters() - lastRightDistanceMeters;
        if (!gyroInputs.gyroConnected) {
            yaw = new Rotation2d(lastYawAngleRads + (rightDelta - leftDelta) / trackwidth);
        }
        odometry.update(yaw, getLeftDistanceMeters(), getRightDistanceMeters());
        Logger.getInstance().recordOutput("Drive/Pose", getPose());
    }

    public void resetPosition(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(gyroInputs.gyroYawPositionRad),
            getLeftDistanceMeters(), getRightDistanceMeters(), pose);
    }

    public void setVoltage(double leftVolts, double rightVolts) {
        driveIO.setVoltage(leftVolts, rightVolts);
    }

    public void setVelocity(double leftVelocity, double rightVelocity) {
        //TODO: make controller
    }
     
    public double getLeftDistanceMeters() {
        return driveInputs.leftPositionRad * wheelRadius;
    }

    public double getRightDistanceMeters() {
        return driveInputs.rightPositionRad * wheelRadius;
    }

    public double getLeftMetersPerSecond() {
        return leftDelta / 0.02; // TODO: Add constant for this
    }

    public double getRightMetersPerSecond() {
        return rightDelta / 0.02;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
}