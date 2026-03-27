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
public class LimelightFrontCenterCommand extends Command {
  private final LimelightSubsystem ll;
  private final LimelightSubsystem ll2;
  private final DriveSubsystem m_drive;
  private final CommandXboxController m_controller;

  private final PIDController pidR = new PIDController(0.01, 0, 0.0008);//0.015, 0, 0.0001
  private final PIDController pidX = new PIDController(0.06, 0, 0.0);
  private final PIDController pidY = new PIDController(0.7, 0, 0.0);

  private final double FINAL_TX = 0.0;
  private final double FINAL_A = 2.8;
  
  private final double MAX_ROT = 2.5;
  private final double MAX_MOVE = 1.5;

  public LimelightFrontCenterCommand(LimelightSubsystem ll, LimelightSubsystem ll2, DriveSubsystem m_drive, CommandXboxController m_controller) {
    this.ll = ll;
    this.ll2 = ll2;
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
    double tx2 = ll2.getLLTX();

    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double ty = ll.getLLTY();
    double ty2 = ll2.getLLTY();
    double ta = ll.getLLTA();
    double ta2 = ll2.getLLTA();

    boolean tv = ll.getLLTV();
    boolean tv2 = ll2.getLLTV();

    if (tv && tv2) {
      double a1 = 90 - tx2;
      double a2 = 90 + tx;
      double a3 = 180 - (a1 + a2);
      double s1 = 20.4; //distance between cameras
      double s2 = (s1 / Math.sin(Math.toRadians(a3))) * Math.sin(Math.toRadians(a1));
      double m = Math.sqrt((Math.pow(s1/2, 2) + Math.pow(s2, 2)) - s1 * s2 * Math.cos(Math.toRadians(a2)));
      double am = Math.toDegrees(Math.asin((Math.sin(Math.toRadians(a2)) * s2) / m));

      rot = pidR.calculate(-(am - 90), FINAL_TX);
      if (Math.abs(tx) < 3.0) {
        rot = 0;
      }
      rot = MathUtil.clamp(rot, -MAX_ROT, MAX_ROT);

      SmartDashboard.putNumber("AM: ", am);
    } else if (tv) {
      rot = pidR.calculate(tx, FINAL_TX);
      //xSpeed = pidX.calculate(tx, FINAL_TX);
      //ySpeed = pidY.calculate(ta, FINAL_A);
      
      //double scale = MathUtil.clamp(1.0-Math.abs(tx) / 25.0, 0.0, 1.0);
      //xSpeed *= scale;
      //ySpeed *= scale;

      if (Math.abs(tx) < 3.0) {
        rot = 0;
      }
      //if (Math.abs(xSpeed) > 0.02) {
      //  xSpeed += Math.copySign(0.05, xSpeed);
      //}
      //if (Math.abs(ySpeed) > 0.02) {
      //  ySpeed += Math.copySign(0.05, ySpeed);
      //}
      rot = MathUtil.clamp(rot, -MAX_ROT, MAX_ROT);
      //xSpeed = MathUtil.clamp(xSpeed, -MAX_MOVE, MAX_MOVE);
      //ySpeed = MathUtil.clamp(ySpeed, -MAX_MOVE, MAX_MOVE);
    } else if (tv2) {
      rot = pidR.calculate(tx2, FINAL_TX);
      if (Math.abs(tx2) < 3.0) {
        rot = 0;
      }
      rot = MathUtil.clamp(rot, -MAX_ROT, MAX_ROT);
    }

   // SmartDashboard.putNumber("Rot: ", rot);
   // SmartDashboard.putNumber("XSPEED: ", xSpeed);
   // SmartDashboard.putNumber("YSPEED: ", ySpeed);
    
    
    m_drive.drive(
      MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband), 
      MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband), 
      -rot, false); //yspeed xspeed
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return (pidR.atSetpoint()/*  && Math.abs(ll.getLLTA() - FINAL_A) < 0.15*/);
  }
}
