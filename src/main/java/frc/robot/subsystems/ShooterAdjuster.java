// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAdjuster extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new ShooterAdjuster. */
  public ShooterAdjuster() {
    motor.getPIDController().setFeedbackDevice(motor.getAbsoluteEncoder());
    motor.getAbsoluteEncoder().setPositionConversionFactor(360);
    motor.getPIDController().setP(0.04);
    motor.getPIDController().setI(0.0001);//i was 0.0002
    motor.getPIDController().setD(0);
    motor.getPIDController().setIZone(2.5);
    motor.getPIDController().setOutputRange(-0.5,0.5);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(40);
    motor.getPIDController().setPositionPIDWrappingMaxInput(360);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter actual angle", motor.getAbsoluteEncoder().getPosition());
  }

  public void setPosition(double angle) {
    motor.setIdleMode(IdleMode.kBrake);
    motor.getPIDController().setReference(angle, CANSparkBase.ControlType.kPosition);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public double getPosition(){
    return motor.getAbsoluteEncoder().getPosition();
  }

  public void goToZero() {
    motor.setIdleMode(IdleMode.kCoast);
    motor.set(0);
  }

}
