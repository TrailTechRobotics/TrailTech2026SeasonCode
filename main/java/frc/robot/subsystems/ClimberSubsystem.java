// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax climb;
  private SparkMax climbFollower;
  private AbsoluteEncoder climbEncoder;
  private PIDController climbPID;
  
  private double climbTraj;

  public ClimberSubsystem(int climbID, int climbFollowerID) {
    climb = new SparkMax(climbID, MotorType.kBrushless);
    climbFollower = new SparkMax(climbFollowerID, MotorType.kBrushless);
    climbFollower.isFollower();
    climbEncoder = climb.getAbsoluteEncoder();
    climbPID = new PIDController(0.1, 0.0, 0.0);
    climbTraj = 0.0;
  }

  @Override
  public void periodic() {
    climb.set(climbPID.calculate(climbEncoder.getPosition(), climbTraj));
  }

  public void setClimbTraj(double traj) {
    climbTraj = traj;
  }

  public boolean atTraj() {
    return climbPID.atSetpoint();
  }
}
