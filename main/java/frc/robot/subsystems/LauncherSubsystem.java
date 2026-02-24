// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMax motor2;
  private final SparkClosedLoopController pidController;

  public LauncherSubsystem(int canId, int canid2) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    motor2 = new SparkMax(canid2, MotorType.kBrushless);
    motor2.isFollower();
    pidController = motor.getClosedLoopController();

    // Configure PID for velocity control
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
        .pid(0.0001, 0, 0)
        .velocityFF(0.000175);
    
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Set velocity in RPM
   */
  public void setVelocity(double velocityRPM) {
    pidController.setReference(velocityRPM, SparkMax.ControlType.kVelocity);
  }

  public void stop() {
    motor.stopMotor();
  }
}