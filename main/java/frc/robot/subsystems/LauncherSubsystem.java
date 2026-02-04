// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  private SparkMax motor;
  //private RelativeEncoder encoder;
  //private PIDController pid;


  public LauncherSubsystem(int mid) {
    motor = new SparkMax(mid, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
  }

  public void move(double speed) {
    motor.set(speed);
  }
}
