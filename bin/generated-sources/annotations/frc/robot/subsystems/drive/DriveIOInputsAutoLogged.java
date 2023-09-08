package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DriveIOInputsAutoLogged extends DriveIO.DriveIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeftPositionRad", leftPositionRad);
    table.put("LeftVelocityRadPerSec", leftVelocityRadPerSec);
    table.put("LeftAppliedVolts", leftAppliedVolts);
    table.put("LeftCurrentAmps", leftCurrentAmps);
    table.put("LeftTempCelcius", leftTempCelcius);
    table.put("RightPositionRad", rightPositionRad);
    table.put("RightVelocityRadPerSec", rightVelocityRadPerSec);
    table.put("RightAppliedVolts", rightAppliedVolts);
    table.put("RightCurrentAmps", rightCurrentAmps);
    table.put("RightTempCelcius", rightTempCelcius);
  }

  @Override
  public void fromLog(LogTable table) {
    leftPositionRad = table.getDouble("LeftPositionRad", leftPositionRad);
    leftVelocityRadPerSec = table.getDouble("LeftVelocityRadPerSec", leftVelocityRadPerSec);
    leftAppliedVolts = table.getDouble("LeftAppliedVolts", leftAppliedVolts);
    leftCurrentAmps = table.getDoubleArray("LeftCurrentAmps", leftCurrentAmps);
    leftTempCelcius = table.getDoubleArray("LeftTempCelcius", leftTempCelcius);
    rightPositionRad = table.getDouble("RightPositionRad", rightPositionRad);
    rightVelocityRadPerSec = table.getDouble("RightVelocityRadPerSec", rightVelocityRadPerSec);
    rightAppliedVolts = table.getDouble("RightAppliedVolts", rightAppliedVolts);
    rightCurrentAmps = table.getDoubleArray("RightCurrentAmps", rightCurrentAmps);
    rightTempCelcius = table.getDoubleArray("RightTempCelcius", rightTempCelcius);
  }

  public DriveIOInputsAutoLogged clone() {
    DriveIOInputsAutoLogged copy = new DriveIOInputsAutoLogged();
    copy.leftPositionRad = this.leftPositionRad;
    copy.leftVelocityRadPerSec = this.leftVelocityRadPerSec;
    copy.leftAppliedVolts = this.leftAppliedVolts;
    copy.leftCurrentAmps = this.leftCurrentAmps.clone();
    copy.leftTempCelcius = this.leftTempCelcius.clone();
    copy.rightPositionRad = this.rightPositionRad;
    copy.rightVelocityRadPerSec = this.rightVelocityRadPerSec;
    copy.rightAppliedVolts = this.rightAppliedVolts;
    copy.rightCurrentAmps = this.rightCurrentAmps.clone();
    copy.rightTempCelcius = this.rightTempCelcius.clone();
    return copy;
  }
}
