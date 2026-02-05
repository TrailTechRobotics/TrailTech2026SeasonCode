// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimelightBackCenterCommand extends Command {
  private final LimelightSubsystem ll;
  private final DriveSubsystem m_drive;
  private final CommandXboxController m_controller;

  private final PIDController pidR = new PIDController(0.015, 0, 0.001);
  private final PIDController pidX = new PIDController(0.06, 0, 0.0);
  private final PIDController pidY = new PIDController(0.7, 0, 0.0);

  private final double FINAL_TX = 0.0;
  private final double FINAL_A = 2.8;
  
  private final double MAX_ROT = 2.5;
  private final double MAX_MOVE = 1.5;

  public LimelightBackCenterCommand(LimelightSubsystem ll, DriveSubsystem m_drive, CommandXboxController m_controller) {
    this.ll = ll;
    this.m_drive = m_drive;
    this.m_controller = m_controller;
    addRequirements(m_drive);

    pidR.setTolerance(0.5);
    pidY.setTolerance(0.15);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    double rot = 0.0;
    double tx = ll.getLLTX();

    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double ty = ll.getLLTY();
    double ta = ll.getLLTA();

    if (ll.getLLTV()) {
      rot = pidR.calculate(tx, FINAL_TX);
      xSpeed = pidX.calculate(tx, FINAL_TX);
      ySpeed = pidY.calculate(ta, FINAL_A);
      
      double scale = MathUtil.clamp(1.0-Math.abs(tx) / 25.0, 0.0, 1.0);
      xSpeed *= scale;
      ySpeed *= scale;

      if (Math.abs(tx) < 3.0) {
        rot = 0;
      }
      if (Math.abs(xSpeed) > 0.02) {
        xSpeed += Math.copySign(0.05, xSpeed);
      }
      if (Math.abs(ySpeed) > 0.02) {
        ySpeed += Math.copySign(0.05, ySpeed);
      }
      rot = MathUtil.clamp(rot, -MAX_ROT, MAX_ROT);
      xSpeed = MathUtil.clamp(xSpeed, -MAX_MOVE, MAX_MOVE);
      ySpeed = MathUtil.clamp(ySpeed, -MAX_MOVE, MAX_MOVE);
    }

   // SmartDashboard.putNumber("Rot: ", rot);
   // SmartDashboard.putNumber("XSPEED: ", xSpeed);
   // SmartDashboard.putNumber("YSPEED: ", ySpeed);

    m_drive.drive(-ySpeed, -xSpeed, -rot, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return (pidR.atSetpoint() && Math.abs(ll.getLLTA() - FINAL_A) < 0.15);
  }
}
