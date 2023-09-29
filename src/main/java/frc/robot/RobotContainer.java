// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.commands.FeedForwardCharacterization;
import static frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  //TODO make final
  private Drive drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive = new Drive(new DriveIOSparkMax(), new GyroIONavX());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        DriveIOSim driveIOSim = new DriveIOSim();
        drive = new Drive(driveIOSim, driveIOSim.getGyroIO());
        break;

      // Replayed robot, disable IO implementations
      default:
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        break;
    }

    // Set up auto routines
    FeedForwardCharacterizationData leftData = new FeedForwardCharacterizationData("Left Data");
    FeedForwardCharacterizationData rightData = new FeedForwardCharacterizationData("Right Data");
    FeedForwardCharacterization ffCharacterization = new FeedForwardCharacterization(drive,
     true,
      leftData,
       rightData,
        drive::setVoltage,
         drive::getLeftMetersPerSecond,
          drive::getRightMetersPerSecond);
    autoChooser.addDefaultOption("Feed Forward", ffCharacterization);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(new RunCommand(() -> {
      double forward = (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 5.0;
      double right = controller.getLeftX() * 4.0;
      drive.setVoltage(forward + right, forward - right);
    }, drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
