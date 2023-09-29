package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
   private final FlywheelIO flywheel;
   private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
   private final RollerIO roller;
   private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

   public Shooter(FlywheelIO flywheelIO, RollerIO rollerIO) {
      this.flywheel = flywheelIO;
      this.roller = rollerIO;
   }

   public enum ShooterPreset {
      HIGH(5500),
      MEDIUM(3500),
      LOW(2500),
      MANUAL(1000);

      private double rpm = 30;
      ShooterPreset(double rpm) {
         setRPM(rpm);
      }

      public void setRPM(double rpm) {
         this.rpm = rpm;
      }

      public double getRPM() {
         return rpm;
      }
   }
}
