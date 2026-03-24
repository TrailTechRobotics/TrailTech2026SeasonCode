// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChuteSubsystem extends SubsystemBase {
  private SparkMax chuteMotor;

  public ChuteSubsystem(int mid) {
    chuteMotor = new SparkMax(mid, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
  }

  public void setChuteSpeed(double speed) {
    chuteMotor.set(speed);
  }

  public double getChuteSpeed() {
    return chuteMotor.get();
  }
}