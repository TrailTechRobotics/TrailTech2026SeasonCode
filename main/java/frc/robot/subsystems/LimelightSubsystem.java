// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class LimelightSubsystem extends SubsystemBase {
  private String ll;
  //Shuffleboard shuffle = new Shuffleboard();

  public LimelightSubsystem(String ll) {
    this.ll = ll;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("tx: ", NetworkTableInstance.getDefault().getTable(ll).getEntry("tx").getDouble(0));
    SmartDashboard.putNumber("ty: ", NetworkTableInstance.getDefault().getTable(ll).getEntry("ty").getDouble(0));
    SmartDashboard.putNumber("ta: ", NetworkTableInstance.getDefault().getTable(ll).getEntry("ta").getDouble(0));
    SmartDashboard.putNumber("tv: ", NetworkTableInstance.getDefault().getTable(ll).getEntry("tv").getDouble(0));
  }

  public double getLLTX() {
    return NetworkTableInstance.getDefault().getTable(ll).getEntry("tx").getDouble(0);
  }

  public double getLLTY() {
    return NetworkTableInstance.getDefault().getTable(ll).getEntry("ty").getDouble(0);
  }

  public double getLLTA() {
    return NetworkTableInstance.getDefault().getTable(ll).getEntry("ta").getDouble(0);
  }

  public boolean getLLTV() {
    return (NetworkTableInstance.getDefault().getTable(ll).getEntry("tv").getDouble(0) == 1.0);
  }
}
