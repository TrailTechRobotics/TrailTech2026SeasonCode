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

public class FuelgrabberSubsystem extends SubsystemBase {

  // Motors
  private final SparkMax slide;
  private final SparkMax scooper;
  private final SparkMax roller;

  // Encoder + PID
  private final RelativeEncoder slideEncoder;
  private final PIDController slidePID;

  // Target position (motor rotations)
  private double slideTraj;

  private final SparkClosedLoopController rollerPidController;
  private final SparkClosedLoopController scooperPidController;
  /**
   * slideID = 9
   * scooperID = 1
   * rollerID = 10
   */
  public FuelgrabberSubsystem(int slideID, int scooperID, int rollerID) {

    slide = new SparkMax(slideID, MotorType.kBrushless);
    scooper = new SparkMax(scooperID, MotorType.kBrushless);
    roller = new SparkMax(rollerID, MotorType.kBrushless);

    // NEO 1.1 internal encoder
    slideEncoder = slide.getEncoder();
    slideEncoder.setPosition(0.0); // zero on boot (or after homing)

    // PID: P only for now
    slidePID = new PIDController(0.03, 0.0, 0.0);
    slidePID.setTolerance(0.05); // motor rotations

    slideTraj = 0.0;

    rollerPidController = roller.getClosedLoopController();
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.closedLoop
        .pid(0.0001, 0, 0)
        .velocityFF(0.000175);
    roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    scooperPidController = scooper.getClosedLoopController();
    SparkMaxConfig scooperConfig = new SparkMaxConfig();
    scooperConfig.closedLoop
        .pid(0.0001, 0, 0)
        .velocityFF(0.000175);
    scooper.configure(scooperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    double output = slidePID.calculate(slideEncoder.getPosition(), slideTraj);
    slide.set(MathUtil.clamp(output, -1.0, 1.0));

    SmartDashboard.putNumber("Slide Position (rot)", slideEncoder.getPosition());
    SmartDashboard.putNumber("Slide Target (rot)", slideTraj);
    SmartDashboard.putNumber("Slide PID Output", output);
  }

  // ---------------- Slide Control ----------------

  
   
  public void setSlideTraj(double traj) {
    slideTraj = traj;
  }

  public boolean slideAtTraj() {
    return slidePID.atSetpoint();
  }

  public void zeroSlideEncoder() {
    slideEncoder.setPosition(0.0);
  }

  // ---------------- Intake Motors ----------------

  public void setScooperSpeed(double speed) {
    scooper.set(speed);
  }

  public void setRollerSpeed(double speed) {
    roller.set(speed);
  }

  public void setRollerVelocity(double velocityRPM) {
    rollerPidController.setReference(velocityRPM, SparkMax.ControlType.kVelocity);
  }

  public void stopRoller() {
    roller.stopMotor();
  }

  public void setScooperVelocity(double velocityRPM) {
    scooperPidController.setReference(velocityRPM, SparkMax.ControlType.kVelocity);
  }

  public void stopScooper() {
    scooper.stopMotor();
  }
}
