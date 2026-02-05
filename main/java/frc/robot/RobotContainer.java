// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.LimelightHelper;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.LimelightFrontCenterCommand;
import frc.robot.commands.LimelightBackCenterCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.HashMap;
import java.util.function.Supplier;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelgrabberSubsystem;
import frc.robot.commands.SlideInCommand;
import frc.robot.commands.SlideOutCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> autoChooser;

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();  
  private final LimelightSubsystem m_limelightFront = new LimelightSubsystem("limelight-elyttr");
  //private final LimelightSubsystem m_limelightBack = new LimelightSubsystem("limelight-elyttrb");
  //private final FuelgrabberSubsystem m_fuelgrabber = new FuelgrabberSubsystem(SET CAN ID, SET CAN ID, SET CAN ID);
  //private final LauncherSubsystem m_launcher = new LauncherSubsystem(SET CAN ID, IDK HOW MANY MOTOR MAYBE SET MORE);
  //private final ClimberSubsystem m_climber = new ClimberSubsystem(SET CAN ID, SET CAN ID);
//UNCOMMENT THESE FOR THE BACK LIMELIGHT, FUELGRABBER STUFF, LAUNCHER STUFF, AND CLIMBER STUFF !!!IMPORTANT!!!
  
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);


  private boolean m_keyX = false;
  private boolean m_keyB = false;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),   
            m_robotDrive));

    //m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.setSpeed(0.0), m_launcher);
    //m_fuelgrabber.setDefaultCommand(new RunCommand(() -> m_fuelgrabber.setScooperSpeed(0.0), m_fuelgrabber);
    //m_fuelgrabber.setDefaultCommand(new RunCommand(() -> m_fuelgrabber.setPinSpeed(0.0), m_fuelgrabber);

//UNCOMMENT THESE WHEN SUBSYSTEMS MADE, SHOULD BE AUTOMATICALLY OVERWRITTEN WHEN BUTTONS ARE PRESSED, THEN RUN WHEN NOT PRESSED
    
    autoChooser = AutoBuilder.buildAutoChooser();
    //autoChooser.setDefaultOption("Default Auto", kDefaultAuto);
    //autoChooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));   IDK WHAT THESE TWO ARE THEY WERE PUT IN DURING LIMELIGHT STUFF IN DECEMEBR
    //m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive));  UNCOMMENT IF EVERTHING BREAKS FSR
    m_driverController.x().whileTrue(new RunCommand(() -> {m_keyX = true;}));
    m_driverController.x().whileTrue(new LimelightFrontCenterCommand(m_limelightFront, m_robotDrive, m_driverController));
    m_driverController.x().whileFalse(new RunCommand(() -> {m_keyX = false;}));
    m_driverController.b().whileTrue(new RunCommand(() -> {m_keyB = true;}));
   // m_driverController.b().whileTrue(new LimelightBackCenterCommand(m_limelightBack, m_robotDrive, m_driverController));
    m_driverController.b().whileFalse(new RunCommand(() -> {m_keyB = false;}));

    //m_driverController.leftTrigger().whileTrue(new RunCommand(() -> {m_launcher.setLauncherSpeed(PUT LAUNCHER BACKWARDS SPEED HERE);}));
    //m_driverController.rightTrigger().whileTrue(new RunCommand(() -> {m_launcher.setLauncherSpeed(PUT LAUNCHER SPEED HERE);}));
  
    //m_driverController.a().whileTrue(new RunCommand(() -> {m_fuelgrabber.setPinSpeed(PUT PIN SPEED HERE);}));
    //m_driverController.y().whileTrue(new RunCommand(() -> {m_fuelgrabber.setPinSpeed(PUT PIN BACKWARDS SPEED HERE);}));
  
    //m_driverController.povLeft().whileTrue(new RunCommand(SlideOutCommand()));
    //m_driverController.povRight().whileTrue(new RunCommand(SlideInCommand()));

    //m_driverController.leftBumber().whileTrue(new RunCommand(() -> {m_fuelgrabber.setScooperSpeed(PUT SCOOPER SPEED HERE);}));
    //m_driverController.rightBumber().whileTrue(new RunCommand(() -> {m_fuelgrabber.setScooperSpeed(PUT SCOOPER BACKWARDS SPEED HERE);}));

 //   m_driverController.povUp().whileTrue(new RunCommand(() -> {m_climber.setTraj(CLIMBER TOP HEIGHT FOR PID HERE);}));
 // IDK IF povDown or povUp IS CORRECT, CHANGE IF NEEDED   m_driverController.povDown().whileTrue(new RunCommand(() -> {m_climber.setTraj(CLIMBER BOTTOM FOR PID HERE);}));
  }  //FIX THE SET 0 TRIGGERS TO CROSS CHECK CONTROLS INSTEAD OF SPEEDS AND ADD FUNCTIONS IN SUBSYSTEMS
//UNCOMMENT THESE FOR THE COMMAND OF THE STUFF, AFTR THE SUBSYSTEMS ARE MADE

/*
              BUTTON BINDINGS
      Left Joystick : Move
      Right Joystick : Turn
      L3 : Nothing
      R3 : Nothing
      
      Left Bumber : Fuelgrabber Scooper Out  (Pulls fuel off ground)
      Right Bumber : Fuelgrabber Scooper In
      POV Left : Fuelgrabber Out    (Full piece that have the "Scooper")
      POV Right : Fuelgrabber In
      A : Pin In    (Thing to move fuel after scooper picks em up)    (as in roller pin like the kitchen tool cause it looked like that on cad)
      Y : Pin Out

      POV Up : Climber Up
      POV Down : Climber Down

      Left Trigger : Launcher In
      Right Trigger : Launcher Out

      X : Limelight Front Align
      B : Limelight Back Align


      CHANGE HOWEVER NEEDED, MAYBE USE L3 AND R3 FOR LIMELIGHT STUFF IF MORE BUTTONS NEEDED
*/
  
  public boolean m_key(String i) {
    boolean ret = false;
    if (i == "x") {
      ret = m_keyX;
    } else if (i == "b") {
      ret = m_keyB;
    } else {
      ret = false;
    }
    return ret;
  }

  public Command getAutonomousCommand() {
    try{
      return autoChooser.getSelected();
    } catch (Exception e) {
      return null;
    }
  }
}
