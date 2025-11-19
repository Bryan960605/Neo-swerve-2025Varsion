// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swervesubsystem = new SwerveSubsystem();
  public static final CommandXboxController baseJoystick = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /* ============
     *    Driver 
     * ============
    */
    /* Manual Drive */
    DoubleSupplier xSpeedFunc = () -> baseJoystick.getRawAxis(1);
    DoubleSupplier ySpeedFunc = () -> baseJoystick.getRawAxis(0);
    DoubleSupplier zSppedFunc = () -> baseJoystick.getRawAxis(4);
    BooleanSupplier isSlowModeFunc = () -> baseJoystick.getHID().getRightTriggerAxis() > 0.4;
    m_swervesubsystem.setDefaultCommand(new ManualDrive(m_swervesubsystem, xSpeedFunc, ySpeedFunc, zSppedFunc, isSlowModeFunc));
    /* Reset Gyro */
    baseJoystick.b().whileTrue(
      Commands.runOnce(()->{
        m_swervesubsystem.resetGyro();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
