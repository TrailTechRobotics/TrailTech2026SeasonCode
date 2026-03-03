// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelgrabberSubsystem;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


//You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FullOuttakeOnCommand extends Command {
  private FuelgrabberSubsystem fuelGrabber;
  private ChuteSubsystem chute;
  private LauncherSubsystem launcher;

  private int scooperVelo;
  private int chuteSpeed;
  private int launcherVelo;

  private LimelightSubsystem m_limelightFront;

 // private int time;

  public FullOuttakeOnCommand(FuelgrabberSubsystem fuelGrabber, ChuteSubsystem chute, LauncherSubsystem launcher, int scooperVelo, int chuteSpeed, LimelightSubsystem m_limelightFront/* , int launcherVelo*/) {
    this.fuelGrabber = fuelGrabber;
    this.chute = chute;
    this.launcher = launcher;

    addRequirements(fuelGrabber, chute, launcher);

    this.scooperVelo = scooperVelo;
    this.chuteSpeed = chuteSpeed;
    //this.launcherVelo = launcherVelo;
    this.m_limelightFront = m_limelightFront;

    //time = 0;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    fuelGrabber.setScooperVelocity(scooperVelo);
    chute.setChuteSpeed(chuteSpeed);
    //launcher.setVelocity(launcherVelo);
    launcher.setVelocity(m_limelightFront.CALCULATESHOOTVELO());
    //time++;
  }

  @Override
  public void end(boolean interrupted) {
    fuelGrabber.stopScooper();
    chute.setChuteSpeed(0);
    launcher.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}