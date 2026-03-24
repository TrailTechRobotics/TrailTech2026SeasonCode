// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeModuleSubsystem;

public class IntakeModuleDownAutoCommand extends Command {
  private IntakeModuleSubsystem m_intakeModule;

  public IntakeModuleDownAutoCommand(IntakeModuleSubsystem m_intakeModule) {
    this.m_intakeModule = m_intakeModule;
    addRequirements(m_intakeModule);
  }

  @Override
  public void initialize() {
    m_intakeModule.setIntakeModuleDown();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
