// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeModuleSubsystem;
import frc.robot.subsystems.FuelgrabberSubsystem;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.IntakeModuleDownAutoCommand;
import frc.robot.commands.FullOuttakeOnVeloCommand;

public class auto1 extends SequentialCommandGroup {
  IntakeModuleSubsystem m_intakeModule;
  FuelgrabberSubsystem m_fuelgrabber;
  ChuteSubsystem m_chute;
  LauncherSubsystem m_launcher;

  public auto1(IntakeModuleSubsystem m_intakeModule, FuelgrabberSubsystem m_fuelgrabber, ChuteSubsystem m_chute, LauncherSubsystem m_launcher) {
    this.m_intakeModule = m_intakeModule;
    this.m_chute = m_chute;
    this.m_fuelgrabber = m_fuelgrabber;
    this.m_launcher = m_launcher;
    IntakeModuleDownAutoCommand a = new IntakeModuleDownAutoCommand(m_intakeModule);
    FullOuttakeOnVeloCommand b = new FullOuttakeOnVeloCommand(m_fuelgrabber, m_chute, m_launcher,
    -4500, 1, 4000);
    addCommands(a, b);
    addRequirements(m_intakeModule, m_fuelgrabber, m_chute, m_launcher);
  }
}
