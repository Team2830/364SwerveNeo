package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private AbsoluteEncoder mAngleEncoder;
    private RelativeEncoder mDriveEncoder;
    private final SparkPIDController mAnglePIDController;
    private final SparkPIDController mDrivePIDController;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleMotor.restoreFactoryDefaults();
        mAngleEncoder = mAngleMotor.getAbsoluteEncoder();
        mAnglePIDController = mAngleMotor.getPIDController();
        mAnglePIDController.setFeedbackDevice(mAngleEncoder);
        mAngleEncoder.setPositionConversionFactor(360);
        mAngleMotor.setInverted(true);
        mAngleEncoder.setInverted(true);
        mAnglePIDController.setPositionPIDWrappingEnabled(true);
        mAnglePIDController.setPositionPIDWrappingMaxInput(360);
        mAnglePIDController.setPositionPIDWrappingMinInput(0);
        mAnglePIDController.setP(0.045);
        mAnglePIDController.setI(0);
        mAnglePIDController.setD(0); //1.5
        mAnglePIDController.setIZone(5);
        mAngleMotor.setIdleMode(IdleMode.kCoast);
        mAngleMotor.setSmartCurrentLimit(20);
        mAngleMotor.burnFlash();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        mDriveEncoder = mDriveMotor.getEncoder();
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setInverted(true);
        mDrivePIDController = mDriveMotor.getPIDController();
        mDrivePIDController.setP(0.032);
        mDrivePIDController.setI(0);
        mDrivePIDController.setD(0);
        mDrivePIDController.setFF(0);
        mDrivePIDController.setIZone(0.5);
        mDriveMotor.setIdleMode(IdleMode.kBrake);
        mDriveMotor.setSmartCurrentLimit(40);
        mDriveEncoder.setPosition(0);
        mDriveEncoder.setVelocityConversionFactor(1/60.0 * 0.18666666666);
        mDriveEncoder.setPositionConversionFactor(0.18666666666);
        mDriveMotor.burnFlash();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAnglePIDController.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(driveDutyCycle.Output);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDrivePIDController.setReference(driveVelocity.Velocity, ControlType.kVelocity, 0, driveVelocity.FeedForward, ArbFFUnits.kVoltage);
        }
    }

    public Rotation2d getCANandCoder(){
        return Rotation2d.fromRotations(mAngleEncoder.getPosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANandCoder().getRotations() - angleOffset.getRotations();
        mAnglePIDController.setReference(absolutePosition, ControlType.kPosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveEncoder.getVelocity(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromDegrees(mAngleEncoder.getPosition())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveEncoder.getPosition(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromDegrees(mAngleEncoder.getPosition())
        );
    }
}