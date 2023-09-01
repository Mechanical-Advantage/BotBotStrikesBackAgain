package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    // constants
    public static double trackwidth = 1.3; // meters (this is made up)
    public static double encoderAfterReduction = 1.0 / ((9.0 / 62.0) * (18.0 / 30.0));
    public static double wheelDiameter = 5.8074934358;

    private DriveIO driveIO;
    private DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    
    private DifferentialDriveKinematics kinematics;
    private double leftDistanceMeters = 0.0, rightDistanceMeters = 0.0;

    private Pose2d pose = new Pose2d();

    public Drive(DriveIO driveIO, GyroIO gyroIO) {
        this.driveIO = driveIO;
        this.gyroIO = gyroIO;

        kinematics = new DifferentialDriveKinematics(trackwidth);
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        driveIO.updateInputs(driveInputs);
        Logger.getInstance().processInputs("Drive/Drive", driveInputs);

        
    }
}