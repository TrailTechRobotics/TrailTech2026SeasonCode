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

public class HopperSubsystem extends SubsystemBase {
  private final SparkMax roller;

  private final SparkClosedLoopController rollerPidController;

  public HopperSubsystem(int rollerID) {
    roller = new SparkMax(rollerID, MotorType.kBrushless);

    rollerPidController = roller.getClosedLoopController();
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.closedLoop
        .pid(0.0001, 0, 0)
        .velocityFF(0.000175);
    roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void setRollerSpeed(double speed) {
    roller.set(speed);
  }

  public void setRollerVelocity(double velocityRPM) {
    rollerPidController.setReference(velocityRPM, SparkMax.ControlType.kVelocity);
  }

  public void stopRoller() {
    roller.stopMotor();
  }
}
