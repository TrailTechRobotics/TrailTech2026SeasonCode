// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakeModuleConstants;

public class IntakeModuleSubsystem extends SubsystemBase {
  
  private final SparkMax intakeModuleMotor;
  private final RelativeEncoder intakeModuleEncoder;
  private final PIDController intakeModulePID;

  private double intakeModulePIDTrajectory;

  public IntakeModuleSubsystem(int intakeModuleMotorId) {
    this.intakeModuleMotor = new SparkMax(intakeModuleMotorId, MotorType.kBrushless);
    this.intakeModuleEncoder = intakeModuleMotor.getEncoder();
    this.intakeModuleEncoder.setPosition(0.0);
    this.intakeModulePID = new PIDController(0.02, 0.0, 0.0);
    this.intakeModulePID.setTolerance(0.1);
    this.intakeModulePIDTrajectory = 0.0;
  }

  @Override
  public void periodic() {
    double motorOutput = intakeModulePID.calculate(intakeModuleEncoder.getPosition(), intakeModulePIDTrajectory);
    intakeModuleMotor.set(MathUtil.clamp(motorOutput, -1.0, 1.0));
  }

  public void setIntakeModulePosition(double position) {
    intakeModulePIDTrajectory = MathUtil.clamp(position, IntakeModuleConstants.UP_POSITION, IntakeModuleConstants.DOWN_POSITION);
  }

  public void setIntakeModuleUp() {
    intakeModulePIDTrajectory = IntakeModuleConstants.UP_POSITION;
  }

  public void setIntakeModuleDown() {
    intakeModulePIDTrajectory = IntakeModuleConstants.DOWN_POSITION;
  }

  public boolean intakeModuleAtTrajectory() {
    return Math.abs(intakeModuleEncoder.getPosition() - intakeModulePIDTrajectory) < 0.15;
  }
}
