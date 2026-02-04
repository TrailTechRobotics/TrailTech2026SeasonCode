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

public class FuelgrabberSubsystem extends SubsystemBase {
  private SparkMax slide;
  private SparkMax scooper;
  private SparkMax pin;
  private AbsoluteEncoder slideEncoder;
  private PIDController slidePID;
  
  private double slideTraj;

  public FuelgrabberSubsystem(int slideID, int scooperID, int pinID) {
    slide = new SparkMax(slideID, MotorType.kBrushless);
    scooper = new SparkMax(scooperID, MotorType.kBrushless);
    pin = new SparkMax(pinID, MotorType.kBrushless);
    slideEncoder = slide.getAbsoluteEncoder();
    slidePID = new PIDController(0.1, 0.0, 0.0);

    slideTraj = 0.0;
  }

  @Override
  public void periodic() {
    slide.set(slidePID.calculate(slideEncoder.getPosition(), slideTraj));
  }

  public void setSlideTraj(double traj) {
    slideTraj = traj;
  }

  public boolean slideAtTraj() {
    return slidePID.atSetpoint();
  }

  public void setScooperSpeed(double speed) {
    scooper.set(speed);
  }

  public void setPinSpeed(double speed) {
    pin.set(speed);
  }
}
