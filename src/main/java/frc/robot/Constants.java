// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final double robotLength = 0.66;
  public static final double robotWidth = 0.66;
  public static double setMaxOutput(double value, double maxOutput){
    if(value > maxOutput){
      return maxOutput;
    }
    else if(-value < -maxOutput){
      return -maxOutput;
    }
    else{
      return value;
    }
  }
  public static final class SwerveModuleConstants{
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveMotorGearRatio = 1/6.75;
    public static final double turningMotorGearRatio = 1.0/(150/7);
    public static final double driveEncoderRot2Meter = driveMotorGearRatio*Math.PI*wheelDiameter;
    public static final double turningEncoderRot2Rad = turningMotorGearRatio*2*Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter/60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad/60.0;
    public static final double turningMotorkP = 0.005;
    public static final double turningMotorKI = 0;
    public static final double turningMotorKD = 0;
    public static final double drivingMotorKP = 0;
    public static final double drivingMotorKI = 0;
    public static final double drivingMotorKD = 0;
    public static final double maxDriveMotorSpeed = 4.0;
    public static final double maxAngularVelocity = 3.0 * Math.PI;
  }

  public static final class SwerveConstants{
    public static final int leftFrontDriveID = 29;
    public static final int rightFrontDriveID = 19;
    public static final int leftRearDriveID = 15;
    public static final int rightRearDriveID = 16;

    public static final int leftFrontTurningID = 21;
    public static final int rightFrontTurningID = 17;
    public static final int leftRearTurningID = 22;
    public static final int rightRearTurningID = 26;

    public static final int leftFrontCANCoderID = 43;
    public static final int rightFrontCANCoderID = 42;
    public static final int leftRearCANCoderID = 44;
    public static final int rightRearCANCoderID = 41;

    public static final double leftFrontOffset = 0.4052734375;
    public static final double rightFrontOffset = 0.141845703125;
    public static final double leftRearOffset = -0.11767578125;
    public static final double rightRearOffset = 0.174072265625;

    public static final boolean leftFrontdriveMotorReversed = true;
    public static final boolean leftFrontTurningMotorReversed = true;
    public static final boolean rightFrontDriveMotorReversed = true;
    public static final boolean rightfrontTurningMotorReversed = true;
    public static final boolean leftRearDriveMotorreversed = false;
    public static final boolean leftRearTurningMotorReversed = true;
    public static final boolean rightRearDriveMotorReversed = false;
    public static final boolean rightRearTurningMotorReversed = true;
    
    public static double joysickValue(double value, double mineOutput){
      if(Math.abs(value) < mineOutput){
        return 0;
      }
      else{
        return value;
      }
    }

    public static final int gyroID = 10;
    //front left, front right, rear left, rear right
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(robotLength/2, robotWidth/2), 
      new Translation2d(robotLength/2, -robotWidth/2), 
      new Translation2d(-robotLength/2, robotWidth/2),
      new Translation2d(-robotLength/2, -robotWidth/2)
  );
  }
}
