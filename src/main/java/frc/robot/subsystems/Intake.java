// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private TalonFX topIntake = new TalonFX(13);
    private TalonFX bottomIntake = new TalonFX(14 );

    public Intake(){
        topIntake.setInverted(false);
        bottomIntake.setInverted(false);
    }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  
  public void setSpeed(double speed){
    topIntake.set(speed);
    bottomIntake.set(speed);
  }

  public void setOutput(double voltage){
    topIntake.setVoltage(voltage);
    bottomIntake.setVoltage(voltage);
  }
   
}
