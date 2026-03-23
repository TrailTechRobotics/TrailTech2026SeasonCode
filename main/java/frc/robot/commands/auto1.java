// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FuelgrabberSubsystem;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.SlideOutAutoCommand;
import frc.robot.commands.FullOuttakeOnVeloCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class auto1 extends SequentialCommandGroup {
  LauncherSubsystem m_launcher;
  FuelgrabberSubsystem m_fuelgrabber;
  ChuteSubsystem m_chute;
  /** Creates a new auto1. */
  public auto1(FuelgrabberSubsystem m_fuelgrabber, ChuteSubsystem m_chute, LauncherSubsystem m_launcher) {
    this.m_chute = m_chute;
    this.m_fuelgrabber = m_fuelgrabber;
    this.m_launcher = m_launcher;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SlideOutAutoCommand a = new SlideOutAutoCommand(m_fuelgrabber);
    FullOuttakeOnVeloCommand b = new FullOuttakeOnVeloCommand(m_fuelgrabber, m_chute, m_launcher,
    -4500, 1, 4000);
    addCommands(a, b);
    addRequirements(m_fuelgrabber, m_chute, m_launcher);
  }
}
