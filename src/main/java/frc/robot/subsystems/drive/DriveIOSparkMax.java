package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;
import frc.robot.util.SparkMAXBurnManager;

public class DriveIOSparkMAX implements DriveIO {

    private final boolean leftInverted;
    private final boolean rightInverted;
    private final CANSparkMax leftLeader;
    private final CANSparkMax leftFollower;
    private final CANSparkMax rightLeader;
    private final CANSparkMax rightFollower;

    private final double afterEncoderReduction;
    private RelativeEncoder leftInternalEncoder;
    private RelativeEncoder rightInternalEncoder;

    public DriveIOSparkMAX () {
        afterEncoderReduction = 1.0 / ((9.0 / 62.0) * (18.0 / 30.0));
        leftInverted = true;
        rightInverted = false;
        leftLeader = new CANSparkMax(3, MotorType.kBrushless);
        leftFollower = new CANSparkMax(12, MotorType.kBrushless);
        rightLeader = new CANSparkMax(16, MotorType.kBrushless);
        rightFollower = new CANSparkMax(15, MotorType.kBrushless);

        leftInternalEncoder = leftLeader.getEncoder();
        rightInternalEncoder = rightLeader.getEncoder();
        if (SparkMAXBurnManager.shouldBurn()) {
            leftLeader.restoreFactoryDefaults();
            leftFollower.restoreFactoryDefaults();
            rightLeader.restoreFactoryDefaults();
            rightFollower.restoreFactoryDefaults();
        }

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        leftLeader.setInverted(leftInverted);
        rightLeader.setInverted(rightInverted);

        leftLeader.enableVoltageCompensation(12.0);
        rightLeader.enableVoltageCompensation(12.0);

        leftLeader.setSmartCurrentLimit(30);
        leftFollower.setSmartCurrentLimit(30);
        rightLeader.setSmartCurrentLimit(30);
        rightFollower.setSmartCurrentLimit(30);

        
        leftLeader.setCANTimeout(0);
        leftFollower.setCANTimeout(0);
        rightLeader.setCANTimeout(0);
        rightFollower.setCANTimeout(0);

        if (SparkMAXBurnManager.shouldBurn()) {
            leftLeader.burnFlash();
            leftFollower.burnFlash();
            rightLeader.burnFlash();
            rightFollower.burnFlash();
        }
    }
    
    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftPositionRad =
            Units.rotationsToRadians(leftInternalEncoder.getPosition())
                / afterEncoderReduction;
        inputs.rightPositionRad =
            Units.rotationsToRadians(rightInternalEncoder.getPosition())
                / afterEncoderReduction;
        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            leftInternalEncoder.getVelocity()) / afterEncoderReduction;
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            rightInternalEncoder.getVelocity()) / afterEncoderReduction;
        inputs.leftAppliedVolts =
            leftLeader.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightAppliedVolts =
            rightLeader.getAppliedOutput() * RobotController.getBatteryVoltage();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
        leftLeader.setIdleMode(mode);
        leftFollower.setIdleMode(mode);
        rightLeader.setIdleMode(mode);
        rightFollower.setIdleMode(mode);
    }
}
