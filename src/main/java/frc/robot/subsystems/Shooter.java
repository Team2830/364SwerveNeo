// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax topLeft = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax bottomLeft = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax topRight = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax bottomRight = new CANSparkMax(12, MotorType.kBrushless);
  public Shooter() {
    topLeft.setInverted(false);
    topRight.setInverted(true);
    bottomLeft.setInverted(false);
    bottomRight.setInverted(true);

    bottomLeft.setIdleMode(IdleMode.kBrake);
    bottomRight.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("Top Amp Speed", .2);
    SmartDashboard.putNumber("Bottom Amp Speed", .3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

    public void setTopMotorVolts(double volts){
    topLeft.setVoltage(volts);
    topRight.setVoltage(volts);

    }

    public void setBottomMotorVolts(double volts){
      bottomLeft.setVoltage(volts);
      bottomRight.setVoltage(volts);
    }

    public void shooterOff(){
      setBottomMotorVolts(0);
      setTopMotorVolts(0);
    }

    public void ampShot() {
      setTopMotorVolts(SmartDashboard.getNumber("Top Amp Speed", .2)*12.0);
      setBottomMotorVolts(SmartDashboard.getNumber("Bottom Amp Speed", .3)*12.0);
    }
  }
