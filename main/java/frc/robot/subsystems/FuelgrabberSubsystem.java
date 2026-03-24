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
  private final SparkMax scooper;
  private final SparkMax roller;

  private final SparkClosedLoopController rollerPidController;
  private final SparkClosedLoopController scooperPidController;

  public FuelgrabberSubsystem(int scooperID, int rollerID) {
    scooper = new SparkMax(scooperID, MotorType.kBrushless);
    roller = new SparkMax(rollerID, MotorType.kBrushless);

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
  public void periodic() {}

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
