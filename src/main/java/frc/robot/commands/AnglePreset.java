// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AnglePreset extends InstantCommand {
  Swerve swerve;
  double redAngle;
  double blueAngle;

  public AnglePreset(Swerve swerve, double redAngle, double blueAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.redAngle = redAngle;
    this.blueAngle = blueAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
      swerve.setDesiredAngle(redAngle);
    }
    else{
      swerve.setDesiredAngle(blueAngle);
    }
  }
}
