// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class LauncherSubsystem extends SubsystemBase {
  private final SparkMax primaryMotor;
  private final SparkMax secondaryMotor;
  private final SparkClosedLoopController pidController;

  private int launchVelo = 5000;

  public LauncherSubsystem(int primaryMotorID, int secondaryMotorID) {
    primaryMotor = new SparkMax(primaryMotorID, MotorType.kBrushless);
    secondaryMotor = new SparkMax(secondaryMotorID, MotorType.kBrushless);
    secondaryMotor.isFollower();
    pidController = primaryMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
    .pid(0.0002, 0, 0)
    .velocityFF(0.000175);
    primaryMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override 
  public void periodic() {
    SmartDashboard.putNumber("LAUNCHER VELO", launchVelo);
  }

  public void setVelocity(double velocityRPM) {
    pidController.setReference(velocityRPM, SparkMax.ControlType.kVelocity);
  }

  public void stop() {
    primaryMotor.stopMotor();
  }

  public void setVelocityByVelo() {
    pidController.setReference(launchVelo, SparkMax.ControlType.kVelocity);
  }

  public void increaseVelo() {
    launchVelo += (launchVelo + 10) <= 6000 ? 10 : 0;
  }

  public void decreaseVelo() {
    launchVelo -= (launchVelo - 10) >= 0 ? 10 : 0;
  }

  public void setVelo(int velo) {
    if (velo < 6000 && velo >= 0) {
      launchVelo = velo;
    }
  }
}