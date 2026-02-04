// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelgrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SlideInCommand extends Command {
  private FuelgrabberSubsystem fuelGrabber;
  
  private double inPos;

  public SlideInCommand(FuelgrabberSubsystem fuelGrabber) {
    this.fuelGrabber = fuelGrabber;
    addRequirements(fuelGrabber);
    inPos = 0.0; // POSTIION OF BALGRABER WHNE THING IS OUT
  }

  
  @Override
  public void initialize() {
    fuelGrabber.setSlideTraj(inPos);
  }

  
  @Override
  public void execute() {}

 
  @Override
  public void end(boolean interrupted) {}

 
  @Override
  public boolean isFinished() {
    return fuelGrabber.slideAtTraj();
  }
}