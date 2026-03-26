// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

//You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FullOuttakeCommand extends Command {
  private HopperSubsystem hopper;
  private ChuteSubsystem chute;
  private LauncherSubsystem launcher;

  private int rollerVelo;
  private int chuteSpeed;
  private int launcherVelo;

  private LimelightSubsystem m_limelightFront;

  public FullOuttakeCommand(HopperSubsystem hopper, ChuteSubsystem chute, LauncherSubsystem launcher, int rollerVelo, int chuteSpeed, LimelightSubsystem m_limelightFront/* , int launcherVelo*/) {
    this.hopper = hopper;
    this.chute = chute;
    this.launcher = launcher;

    addRequirements(hopper, chute, launcher);

    this.rollerVelo = rollerVelo;
    this.chuteSpeed = chuteSpeed;
    //this.launcherVelo = launcherVelo;

    this.m_limelightFront = m_limelightFront;
  }

  @Override
  public void initialize() {
    hopper.setRollerVelocity(rollerVelo);
    chute.setChuteSpeed(chuteSpeed);
    //launcher.setVelocity(launcherVelo);
    launcher.setVelocity(m_limelightFront.CALCULATESHOOTVELO());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    hopper.stopRoller();
    chute.setChuteSpeed(0);
    launcher.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}