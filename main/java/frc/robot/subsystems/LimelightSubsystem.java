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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;


import java.lang.Math;

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

  public double getLLDISTANCE() {
    double h1 = 30; //LENS HEIGHT IN INCHES
    double h2 = 44.25; //APRIL TAG HEIGHT IN INCHES
    double a = getLLTY() * (3.14159 / 180.0);
    double d = (h2 - h1) / (Math.tan(a));
    return d; //RETURNS IN INCHES
  }

  public double CALCULATESHOOTVELO() {
    LimelightHelpers.setFiducial3DOffset("elyttr",
      -0.584,
      0.0,
      0.0
    );

    double velo = 1.0;
    double d = Math.abs(Units.inchesToMeters(getLLDISTANCE()));
    velo = d * 9.81;
    velo /= Math.sin(2 * 70); //USES LAUNCH ANGLE
    velo = Math.sqrt(velo);

    LimelightHelpers.setFiducial3DOffset("elyttr",
      0.0,
      0.0,
      0.0
    );

    return Math.floor(velo); //RETURNS IN METERS
  }//CHANGE TO CALCULATE TO HUB INSTEAD OF APRIL TAG
}
