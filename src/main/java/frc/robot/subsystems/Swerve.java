package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.Swerve.AutonConstants;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    private PIDController mTranslationXController = new PIDController(0.01, 0, 0);
    private PIDController mTranslationYController = new PIDController(0.01, 0, 0);
    private PIDController mAngleController = new PIDController(0.6, 0, 0.01);
    private double desiredAngle = 0;

    public Swerve() {
        gyro = new AHRS(Port.kMXP);
        gyro.reset();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        mAngleController.enableContinuousInput(0, 360);
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner()
    {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                            AutonConstants.TRANSLATION_PID,
                                            // Translation PID constants
                                            AutonConstants.ANGLE_PID,
                                            // Rotation PID constants
                                            4.5,
                                            // Max module speed, in m/s
                                            Math.hypot(10.375, 10.375),
                                            // Drive base radius in meters. Distance from robot center to furthest module.
                                            new ReplanningConfig()
                                            // Default path replanning config. See the API for the options here
        ),
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
                                    );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void drive(Translation2d translation, double headingX, double headingY, boolean fieldRelative, boolean isOpenLoop) {
        if(Math.hypot(headingX, headingY) > 0.3) {
            desiredAngle = Math.atan2(headingX, headingY);
        }

        double rotation = mAngleController.calculate(getGyroYaw().getDegrees(), desiredAngle);
        rotation = MathUtil.clamp(rotation, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void driveWithPID(Translation2d translation, double headingX, double headingY) {
        if(Math.hypot(headingX, headingY) > 0.3) {
            desiredAngle = Math.atan2(headingX, headingY);
        }

        double rotation = mAngleController.calculate(getGyroYaw().getDegrees(), desiredAngle);
        rotation = MathUtil.clamp(rotation, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);
        double x = mTranslationXController.calculate(getFieldOrientedChassisSpeeds().vxMetersPerSecond, translation.getX() * Constants.Swerve.maxSpeed)
            + translation.getX();
        double y = mTranslationXController.calculate(getFieldOrientedChassisSpeeds().vyMetersPerSecond, translation.getX() * Constants.Swerve.maxSpeed)
            + translation.getY();

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    x, 
                                    y, 
                                    rotation, 
                                    getHeading()
                                ));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public ChassisSpeeds getRobotVelocity() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldOrientedChassisSpeeds() {
        ChassisSpeeds robotOrientedSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotOrientedSpeeds, getGyroYaw());
    }

    public void setChassisSpeeds(ChassisSpeeds speeds)
    {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);

        for(SwerveModule module : mSwerveMods)
        {
            module.setDesiredState(states[module.moduleNumber], false);
        }
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw()).unaryMinus();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setDesiredAngle(double desiredAngle) {
        this.desiredAngle = desiredAngle;
    }

    public double getDesiredAngle() {
        return this.desiredAngle;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANandCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}