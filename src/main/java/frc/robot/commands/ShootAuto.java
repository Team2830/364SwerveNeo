// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAuto extends SequentialCommandGroup {
  /** Creates a new ShootAuto. */
  public ShootAuto(Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new WaitCommand(1), new PrepareToShoot(shooter)), 
    new ParallelDeadlineGroup(new WaitCommand(3), new Shoot(shooter, 12, 12)), 
    new InstantCommand(()-> shooter.setTopMotorVolts(0)),
    new InstantCommand(()-> shooter.setTopMotorVolts(0)));
  }
}
