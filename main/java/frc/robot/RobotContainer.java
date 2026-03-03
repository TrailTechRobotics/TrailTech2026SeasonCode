// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.*;
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
import frc.robot.commands.LimelightFrontCenterAutoCommand;
import frc.robot.commands.LimelightBackCenterCommand;
import frc.robot.commands.RollerOffInstantCommand;
import frc.robot.commands.RollerOnInstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.HashMap;
import java.util.function.Supplier;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FuelgrabberSubsystem;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.commands.SlideInCommand;
import frc.robot.commands.SlideOutCommand;
import frc.robot.commands.FullOuttakeCommand;
import frc.robot.commands.FullOuttakeOnCommand;

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
  private final LimelightSubsystem m_limelightBack = new LimelightSubsystem("limelight-elyttrb");
  private final FuelgrabberSubsystem m_fuelgrabber = new FuelgrabberSubsystem(9, 1, 10);
  private final LauncherSubsystem m_launcher = new LauncherSubsystem(11, 12); // changed LauncherSubsystem to ChuteSubsystem.   fix canIDs
  //private final ClimberSubsystem m_climber = new ClimberSubsystem(22, 33);
  private final ChuteSubsystem m_chute = new ChuteSubsystem(3);
//UNCOMMENT THESE FOR THE BACK LIMELIGHT, FUELGRABBER STUFF, LAUNCHER STUFF, AND CLIMBER STUFF !!!IMPORTANT!!!
  
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  private boolean m_keyX = false;
  private boolean m_keyB = false;

  private int launcherVelo = 4400;
  /**path
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //NAMED COMMAND
    NamedCommands.registerCommand("RollerOff", new RollerOffInstantCommand(m_fuelgrabber));
    NamedCommands.registerCommand("RollerOn", new RollerOnInstantCommand(m_fuelgrabber));
    NamedCommands.registerCommand("SlideIn", new SlideInCommand(m_fuelgrabber));
    NamedCommands.registerCommand("SlideOut", new SlideOutCommand(m_fuelgrabber));
    NamedCommands.registerCommand("FullOuttakeOn", new FullOuttakeOnCommand(
      m_fuelgrabber, m_chute, m_launcher,
      -4500, 1, m_limelightFront//, launcherVelo
    ));
    NamedCommands.registerCommand("LimelightFrontCenter", new LimelightFrontCenterAutoCommand(m_limelightFront, m_robotDrive));

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
                false),  //FIELD RELATIVE 
            m_robotDrive));

    //m_launcher.setDefaultCommand(new RunCommand(() -> {m_launcher.setSpeed(0.0);}, m_launcher);
    //m_fuelgrabber.setDefaultCommand(new RunCommand(() -> {m_fuelgrabber.setScooperSpeed(0.0);}, m_fuelgrabber);
    //m_fuelgrabber.setDefaultCommand(new RunCommand(() -> {m_fuelgrabber.setPinSpeed(0.0);}, m_fuelgrabber);

//UNCOMMENT THESE WHEN SUBSYSTEMS MADE, SHOULD BE AUTOMATICALLY OVERWRITTEN WHEN BUTTONS ARE PRESSED, THEN RUN WHEN NOT PRESSED  
    autoChooser = AutoBuilder.buildAutoChooser();//new SendableChooser<>();
    //***autoChooser.setDefaultOption("No Auto", new InstantCommand());
    //***autoChooser.setDefaultOption("Default Auto", kDefaultAuto);
    //***autoChooser.addOption("My Auto", "Test Auto 1.auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //SmartDashboard.putNumber("LAUNCHER VELOCITY", launcherVelo);
    //launcherVelo = SmartDashboard.getNumber("LAUNCHER VELOCITY");
  }

  private void configureButtonBindings() {
    //      RESET HEADINGS
    //m_driverController.povDown().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive))
    //  .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive)); 

    //      LIMELIGHT
    m_driverController.a().whileTrue(new LimelightFrontCenterCommand(m_limelightFront, m_robotDrive, m_driverController));
   // m_driverController.b().whileTrue(new LimelightBackCenterCommand(m_limelightBack, m_robotDrive, m_driverController));
    
    //      launcher velocity control
    m_driverController.rightBumper()
    .whileTrue(new RunCommand(() -> m_launcher.setVelocityByVelo(), m_launcher))//m_launcher.setVelocity(4200), m_launcher))  
    .onFalse(new InstantCommand(() -> m_launcher.stop(), m_launcher));
    m_driverController.x()                                                //-0.646 for offset below if needed
    //.onTrue(new InstantCommand(() -> m_limelightFront.setLLOFFSET(0.508, 0, 0))) //APRIL TAG OFFSET FOR SHOOTING     BACK THEN SIDE THEN DOWN
    .whileTrue(new RunCommand(() -> {
      m_launcher.setVelocity(m_limelightFront.CALCULATESHOOTVELO());
    }, m_launcher))
    .onFalse(new InstantCommand(() -> m_launcher.stop(), m_launcher));

    //      intake forward
    m_driverController.rightTrigger(0.3)
    .whileTrue(new RunCommand(() -> {m_fuelgrabber.setRollerVelocity(-2800);}))
    .onFalse(new InstantCommand(() -> {m_fuelgrabber.stopRoller();}));

    //      intake reverse
    m_driverController.leftTrigger(0.3)
    .whileTrue(new RunCommand(() -> {m_fuelgrabber.setRollerVelocity(2800);}))
    .onFalse(new InstantCommand(() -> {m_fuelgrabber.stopRoller();}));

    //      Extend slide (Y)
    m_driverController.povUp().whileTrue(new SlideOutCommand(m_fuelgrabber));

    //      Retract slide (A)
    m_driverController.povDown().whileTrue(new SlideInCommand(m_fuelgrabber));
    
    //chute empty to launcher
    m_driverController.leftBumper()
    .whileTrue(new RunCommand(() -> {m_chute.setChuteSpeed(1);}))
    .onFalse(new InstantCommand(() -> {m_chute.setChuteSpeed(0);}));

    //hopper empty to chute
    m_driverController.b()
    .whileTrue(new RunCommand(() -> {m_fuelgrabber.setScooperVelocity(-4500);}))
    .onFalse(new InstantCommand(() -> {m_fuelgrabber.stopScooper();}));

    //      FULL OUTTAKE
    /*m_driverController.y().whileTrue(new FullOuttakeCommand(
      m_fuelgrabber, m_chute, m_launcher,
      -4500, 1, m_limelightFront//, launcherVelo
    ));*/
    m_driverController.y().whileTrue(Commands.startEnd(
      () -> {
        m_fuelgrabber.setScooperVelocity(-4500);
        m_chute.setChuteSpeed(1);
        m_launcher.setVelocity(m_limelightFront.CALCULATESHOOTVELO());
      },
      () -> {
        m_fuelgrabber.stopScooper();
        m_chute.setChuteSpeed(0);
        m_launcher.stop();
      },
      m_launcher, m_fuelgrabber, m_chute, m_limelightFront
    ));

    //      OUTTAKE SPEEDS
    m_driverController.povLeft().whileTrue(new RunCommand(() -> {m_launcher.setVelo(true);}));
    m_driverController.povRight().whileTrue(new RunCommand(() -> {m_launcher.setVelo(false);}));
  }  
/*
              BUTTON BINDINGS (OUTDATED)
      Right Joystick : Turn
      L3 : Nothing
      R3 : Nothing
      
      Left Bumper : Fuelgrabber Scooper Out  (Pulls fuel off ground)
      Right Bumper : Fuelgrabber Scooper In
      POV Left : Fuelgrabber Out    (Full piece that have the "Scooper")
      POV Right : Fuelgrabber In
      A : Intake Retract   (Thing to move fuel after scooper picks em up)    (as in roller pin like the kitchen tool cause it looked like that on cad)
      Y : Intake Extend

      POV Up : Climber Up
      POV Down : Climber Down

      Left Trigger : Intake Out
      Right Trigger : Intake in

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
    //try{
      return autoChooser.getSelected();
    //} catch (Exception e) {
    //  return null;
    //}
  }
}