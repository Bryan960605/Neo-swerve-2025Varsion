// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem m_swerveSubsystem;
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  private final DoubleSupplier xFunc;
  private final DoubleSupplier yFunc;
  private final DoubleSupplier zFunc;
  private final BooleanSupplier slowbtn;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter zLimiter;
  public ManualDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed, BooleanSupplier slowbtn) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.xFunc = xSpeed;
    this.yFunc = ySpeed;
    this.zFunc = zSpeed;
    this.slowbtn = slowbtn;
    xLimiter = new SlewRateLimiter(4);
    yLimiter = new SlewRateLimiter(4);
    zLimiter = new SlewRateLimiter(4);
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xFunc.getAsDouble() * SwerveModuleConstants.maxDriveMotorSpeed;
    ySpeed = yFunc.getAsDouble() * SwerveModuleConstants.maxDriveMotorSpeed;
    zSpeed = zFunc.getAsDouble() * SwerveModuleConstants.maxAngularVelocity;

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
    zSpeed = MathUtil.applyDeadband(zSpeed, 0.1);

    if(slowbtn.getAsBoolean()){
      xSpeed = xSpeed*0.4;
      ySpeed = ySpeed*0.4;
      zSpeed = zSpeed*0.4;
    }else{
      xSpeed = xSpeed*0.9;
      ySpeed = ySpeed*0.9;
      zSpeed = zSpeed*0.9;
    }

    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    zSpeed = zLimiter.calculate(zSpeed);
    m_swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
