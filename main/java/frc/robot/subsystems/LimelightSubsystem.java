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

import java.util.ArrayList;

import java.lang.Math;

public class LimelightSubsystem extends SubsystemBase {
  private String ll;
  //Shuffleboard shuffle = new Shuffleboard();

  private int lastVelo;
  private int dt;

  //private ArrayList<Double> tyAverage;

  public LimelightSubsystem(String ll) {
    this.ll = ll;

    LimelightHelpers.setFiducial3DOffset(ll,
      -0.597,
      0, //OFFSET ON ROBOT
      0
    );

    lastVelo = 0;
    dt = 0;
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

  public void setLLOFFSET(double front, double side, double up) {
    LimelightHelpers.setFiducial3DOffset(ll,
      front,
      side,
      up
    );
  }  //-0.584,

  public double getLLDISTANCE() {
    if (!getLLTV()) {
      return 0;
    }

    /*tyAverage.add(getLLTY());
    if (tyAverage.size() > 5) {
      tyAverage.remove(0);
    }
    double av = 0;
    for(double i : tyAverage) {
      av += i;
    }
    av /= tyAverage.size();*/
    
    double h1 = 18.7; //LENS HEIGHT IN INCHES
    double h2 = 44.25; //APRIL TAG HEIGHT IN INCHES
    double a = -(Math.toRadians(getLLTY() + 1.5));//ROTATION FIX
    a = Math.tan(a);
    if (Math.abs(a) < 0.001) {
      return 0;
    }
    double d = (h2 - h1) / a;

    SmartDashboard.putNumber("LIMELIGHT DISTANCE", d);
    return d; //RETURNS IN INCHES
  }

  public double CALCULATESHOOTVELO() {
    double d = getLLDISTANCE();

    if (!getLLTV() || (d == 0)) {
      dt++;
      lastVelo -= 10 * dt;
      return lastVelo;
    }

    dt = 0;

    double velo = 1.0;
    /*double d = Math.abs(Units.inchesToMeters(getLLDISTANCE()));
    velo = d * 9.81;
    velo /= Math.sin(2 * Math.toRadians(70)); //USES LAUNCH ANGLE
    velo = Math.sqrt(velo);

    velo *= 60;
    velo /= 31.41;
    velo *= 80; //FIX THIS STUFF LATER*/

    //velo = Math.abs(getLLDISTANCE()) * 12.05123 + 2491.92909;        //other way
    //velo = Math.pow(1.00304, Math.abs(getLLDISTANCE())) * 2728.86503;        //other way
    velo = (0.0919939 * Math.pow(Math.abs(d), 2)) - (13.35425 * Math.abs(d)) + 3785.72209;
    //velo = (0.0653108 * Math.pow(Math.abs(d), 2)) - (6.10119 * Math.abs(d)) + 3390.95238;

    velo = /*MathUtil.clamp(velo, 0, 6000);*/(velo > 6000 ? 6000 : velo < 0 ? 0 : velo);//Math.clamp(0, 7000);

    int veloInt = (int) Math.floor(velo); 
    veloInt = (veloInt > lastVelo + 20 && veloInt < lastVelo + 100 ? lastVelo : veloInt < lastVelo - 20 && veloInt > lastVelo - 100 ? lastVelo : veloInt);

    SmartDashboard.putNumber("RPM", veloInt);
  

    lastVelo = veloInt;
    return veloInt; //
  }//CHANGE TO CALCULATE TO HUB INSTEAD OF APRIL TAG
}//156.6688    82.016  76.17
