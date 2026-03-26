// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkClosedLoopController intakeMotorPID;

  public IntakeSubsystem(int intakeMotorId) {
    this.intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);

    intakeMotorPID = intakeMotor.getClosedLoopController();
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.closedLoop
        .pid(0.0001, 0, 0)
        .velocityFF(0.000175);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void setIntakeVelocity(double velocityRPM) {
    intakeMotorPID.setReference(velocityRPM, SparkMax.ControlType.kVelocity);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }
}
