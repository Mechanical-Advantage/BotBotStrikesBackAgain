package frc.robot.subsystems.shooter;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.SparkMAXBurnManager;

public class FlywheelIOSparkMax implements FlywheelIO {
   private static final boolean invertFlywheel = true;
   private static final int currentLimit = 30;
   private CANSparkMax flywheelLeader;
   private CANSparkMax flywheelFollower;
   private SparkMaxPIDController pid;
   private final RelativeEncoder encoder;

   public FlywheelIOSparkMax(){
      flywheelLeader = new CANSparkMax(14, MotorType.kBrushless);
      flywheelFollower = new CANSparkMax(13, MotorType.kBrushless);

      if (SparkMAXBurnManager.shouldBurn()) {
         flywheelLeader.restoreFactoryDefaults();
         flywheelFollower.restoreFactoryDefaults();
         flywheelFollower.follow(flywheelLeader);
      }

      pid = flywheelLeader.getPIDController();
      encoder = flywheelLeader.getEncoder();
      flywheelLeader.setSmartCurrentLimit(currentLimit);
      flywheelFollower.setSmartCurrentLimit(currentLimit);

      flywheelLeader.setInverted(invertFlywheel);


   }
   @Override
   public void updateInputs(FlywheelIOInputs inputs) {
      inputs.velocityRPM = encoder.getVelocity() * ShooterConstants.flywheelEncoderReduction;
      inputs.appliedVolts =
              flywheelLeader.getAppliedOutput() * RobotController.getBatteryVoltage();
      inputs.currentAmps = new double[] {flywheelLeader.getOutputCurrent(),};
   }

   @Override
   public void setVoltage(double volts) {
      flywheelLeader.setVoltage(volts);
   }

   public void setShooterRPM(double rpm) {


   }


}

