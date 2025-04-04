package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase{
    
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final SparkMaxConfig driveMotorConfig;
    private final SparkMaxConfig turningMotorConfig;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPIDController;
    private final PIDController drivingPIDController;

    private final CANcoder absoluteEncoder;
    private final CANcoderConfiguration cancoderConfig;

    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, 
                        int absoluteEncoderID, double absoluteEncoderOffsetDegree){
        absoluteEncoder = new CANcoder(absoluteEncoderID);
        cancoderConfig = new CANcoderConfiguration();

        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        driveMotorConfig = new SparkMaxConfig();
        turningMotorConfig = new SparkMaxConfig();

        driveMotorConfig.inverted(driveMotorReversed);
        turningMotorConfig.inverted(turningMotorReversed);

        driveMotorConfig.idleMode(IdleMode.kBrake);
        turningMotorConfig.idleMode(IdleMode.kBrake);

        cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffsetDegree/360;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        turningPIDController = new PIDController(SwerveModuleConstants.turningMotorkP, SwerveModuleConstants.turningMotorKI, SwerveModuleConstants.turningMotorKD);
        turningPIDController.enableContinuousInput(-180, 180);

        drivingPIDController = new PIDController(SwerveModuleConstants.drivingMotorKP, SwerveModuleConstants.drivingMotorKI, SwerveModuleConstants.drivingMotorKD);
        
        resetEncoders();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition() * SwerveModuleConstants.driveEncoderRot2Meter;
    }
    public double getTurningPosition(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;
    }
    public double getTurnintEncoderPosition(){
        return turningEncoder.getPosition() * SwerveModuleConstants.turningEncoderRot2Rad;
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity() * SwerveModuleConstants.driveEncoderRPM2MeterPerSec;
    }
    public double getTurningVelocity(){
        return turningEncoder.getVelocity() * SwerveModuleConstants.turningEncoderRPM2RadPerSec;
    }
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
        absoluteEncoder.getConfigurator().apply(cancoderConfig);
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningPosition()));
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningPosition()));
    }
     
    public void setDesiredState(SwerveModuleState state){
        state.optimize(getState().angle);
        double drivingOutput = state.speedMetersPerSecond / SwerveModuleConstants.maxDriveMotorSpeed + drivingPIDController.calculate(getState().speedMetersPerSecond, state.speedMetersPerSecond);
        driveMotor.set(drivingOutput);
        turningMotor.set(turningPIDController.calculate(getState().angle.getDegrees(),state.angle.getDegrees()));
    }


    @Override
    public void periodic(){}
}