package frc.robot.subsystems;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.*;


public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule leftFrontModule, rightFrontModule, leftRearModule, rightRearModule;
    private final Pigeon2 gyro = new Pigeon2(gyroID);
    private SwerveDriveOdometry mOdometry;
    private final Field2d field;
    private RobotConfig robotConfig;
    private final Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
    private SwerveModuleState[] myState;
    private StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
    private StructArrayPublisher<SwerveModuleState> targetStatePublisher;
    private StructPublisher<ChassisSpeeds> chassisSpeedPublisher;
    private StructPublisher<Rotation2d> rotationPublisher;
    private StructPublisher<Pose2d> posePublisher;
    public SwerveSubsystem(){
        gyroConfig.withMountPose(new MountPoseConfigs().withMountPoseYaw(78));
        gyro.getConfigurator().apply(gyroConfig);
        leftFrontModule = new SwerveModule(
            leftFrontDriveID, 
            leftFrontTurningID, 
            leftFrontdriveMotorReversed, 
            leftFrontCANCoderID, 
            leftFrontOffset);
        rightFrontModule = new SwerveModule(
            rightFrontDriveID,
            rightFrontTurningID,
            rightFrontDriveMotorReversed, 
            rightFrontCANCoderID, 
            rightFrontOffset);
        leftRearModule = new SwerveModule(
            leftRearDriveID, 
            leftRearTurningID, 
            leftRearDriveMotorreversed, 
            leftRearCANCoderID, 
            leftRearOffset);
        rightRearModule = new SwerveModule(
            rightRearDriveID, 
            rightRearTurningID, 
            rightRearDriveMotorReversed, 
            rightRearCANCoderID, 
            rightRearOffset);

        mOdometry = new SwerveDriveOdometry(swerveKinematics, gyro.getRotation2d(), getModulePosition());

        field = new Field2d();

        resetGyro();

        try{
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }
        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(pathingMoving_Kp, pathingtheta_Ki, pathingtheta_Kd), // Translation PID constants
                    new PIDConstants(pathingtheta_Kp, pathingMoving_Ki, pathingMoving_Kd) // Rotation PID constants
            ),
            robotConfig, // The robot configuration
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

        // // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        moduleStatePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("CurrentStates", SwerveModuleState.struct).publish();

        targetStatePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("TargetStates", SwerveModuleState.struct).publish();

        chassisSpeedPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("CurrentChassisSpeed", ChassisSpeeds.struct).publish();

        rotationPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("CurrentAngle", Rotation2d.struct).publish();

        posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("CurrentPose", Pose2d.struct).publish();
    }

    public void resetGyro(){
        gyro.reset();
    }
    public Rotation2d getRotation2d(){
        return gyro.getRotation2d();
    }
    public SwerveModulePosition[] getModulePosition(){
        return new SwerveModulePosition[]{
            leftFrontModule.getPosition(),
            rightFrontModule.getPosition(),
            leftRearModule.getPosition(),
            rightRearModule.getPosition()
        };
    }
    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            leftFrontModule.getState(),
            rightFrontModule.getState(),
            leftRearModule.getState(),
            rightRearModule.getState()
        };
    }
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        leftFrontModule.setDesiredState(desiredStates[0]);
        rightFrontModule.setDesiredState(desiredStates[1]);
        leftRearModule.setDesiredState(desiredStates[2]);
        rightRearModule.setDesiredState(desiredStates[3]);
    }
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented){
        SwerveModuleState[] states = null;
        if(fieldOriented){
            states = swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, gyro.getRotation2d()));
        }else{
            states = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        }
        myState = states;
        setModuleStates(states);
    }
    public Pose2d getPose(){
        return mOdometry.getPoseMeters();
    }
    public void setPose(Pose2d pose){
        mOdometry.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
    }
    public ChassisSpeeds getChassisSpeed() {
        return swerveKinematics.toChassisSpeeds(getModuleStates());
    }
    public void autoDrive(ChassisSpeeds speeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.01);
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(states);
    }
    @Override
    public void periodic(){
        mOdometry.update(gyro.getRotation2d(), getModulePosition());
        field.setRobotPose(getPose());
        moduleStatePublisher.set(getModuleStates());
        targetStatePublisher.set(myState);
        chassisSpeedPublisher.set(getChassisSpeed());
        rotationPublisher.set(getRotation2d());
        posePublisher.set(getPose());
    }
  
}