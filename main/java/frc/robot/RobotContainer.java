// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.LauncherConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LimelightFrontCenterCommand;
import frc.robot.commands.LimelightFrontCenterAutoCommand;
import frc.robot.commands.LimelightBackCenterCommand;
import frc.robot.commands.RollerOffInstantCommand;
import frc.robot.commands.RollerOnInstantCommand;
import frc.robot.commands.FullOuttakeCommand;
import frc.robot.commands.FullOuttakeOnCommand;
import frc.robot.commands.FullOuttakeOnVeloCommand;
import frc.robot.commands.auto1;
import frc.robot.commands.IntakeModuleDownAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.IntakeModuleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.List;
import java.util.HashMap;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
  private SendableChooser<Command> autoChooser;

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();  
  private final LimelightSubsystem m_limelightFront = new LimelightSubsystem("limelight-elyttr");
  private final LimelightSubsystem m_limelightBack = new LimelightSubsystem("limelight-elyttrb");
  private final IntakeModuleSubsystem m_intakeModule = new IntakeModuleSubsystem(9);
  private final HopperSubsystem m_hopper = new HopperSubsystem(10);
  private final ChuteSubsystem m_chute = new ChuteSubsystem(3);
  private final LauncherSubsystem m_launcher = new LauncherSubsystem(11, 12);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(1);


  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_copilotController = new CommandXboxController(OIConstants.kCopilotControllerPort);

  public RobotContainer() {
    //Camera Offsets
    m_limelightBack.setLLOFFSET(LimelightConstants.APRIL_TAG_FRONT_OFFSET,0,0);
    m_limelightFront.setLLOFFSET(LimelightConstants.APRIL_TAG_FRONT_OFFSET,0,0);

    //Named Commands For Auto
    NamedCommands.registerCommand("IntakeModuleDown", new IntakeModuleDownAutoCommand(m_intakeModule));
    NamedCommands.registerCommand("RollerOff", new RollerOffInstantCommand(m_hopper));
    NamedCommands.registerCommand("RollerOn", new RollerOnInstantCommand(m_hopper));
    NamedCommands.registerCommand("FullOuttakeOn", new FullOuttakeOnCommand(
      m_hopper, m_chute, m_launcher,
      -4500, 1, m_limelightFront
    ));
    NamedCommands.registerCommand("LimelightFrontCenter", new LimelightFrontCenterAutoCommand(m_limelightFront, m_limelightBack, m_robotDrive));
    NamedCommands.registerCommand("FullOuttakeOnVelo", new FullOuttakeOnVeloCommand(
      m_hopper, m_chute, m_launcher,
      -4500, 1, 4000
    ));

    //Configure the button bindings
    configureButtonBindings();

    //Configure default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> 
                m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false),
                //m_launcher.setVelocity(Math.abs(-MathUtil.applyDeadband(m_copilotController.getLeftY(), OIConstants.kDriveDeadband)) * LauncherConstants.MAX_VELOCITY);
              
            m_robotDrive));

    //Autochooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureButtonBindings() {
    //Driver Reset Headings
    m_driverController.start()
    .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive))
    .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)); 

    //Driver Limelight
    m_driverController.leftBumper().whileTrue(new LimelightFrontCenterCommand(m_limelightFront, m_limelightBack, m_robotDrive, m_driverController));
    
    //Copilot Limelight
    m_copilotController.povDown().whileTrue(new LimelightFrontCenterCommand(m_limelightFront, m_limelightBack, m_robotDrive, m_driverController));
     
    //Driver Intake Module Down
    m_driverController.povUp().onTrue(new InstantCommand(() -> m_intakeModule.setIntakeModuleDown(), m_intakeModule));

    //Driver Intake Module Up
    m_driverController.povDown().onTrue(new InstantCommand(() -> m_intakeModule.setIntakeModuleUp(), m_intakeModule));

    //Driver Intake Forward
    m_driverController.rightTrigger(0.3)
    .whileTrue(new RunCommand(() -> {m_intakeSubsystem.setIntakeVelocity(-3800);}))
    .onFalse(new InstantCommand(() -> {m_intakeSubsystem.stopIntake();}));

    //Driver Intake Reverse
    m_driverController.leftTrigger(0.3)
    .whileTrue(new RunCommand(() -> {m_intakeSubsystem.setIntakeVelocity(3800);}))
    .onFalse(new InstantCommand(() -> {m_intakeSubsystem.stopIntake();}));

    //Driver Full Launch System With Limelight
    m_driverController.rightBumper().whileTrue(Commands.startEnd(
      () -> {
        m_hopper.setRollerVelocity(-5500);
        m_chute.setChuteSpeed(1);
        m_launcher.setVelocity(
          (m_limelightFront.getLLTV() && m_limelightBack.getLLTV()) ? (m_limelightFront.CALCULATESHOOTVELO() + m_limelightBack.CALCULATESHOOTVELO()) / 2
          : (m_limelightFront.getLLTV() ? m_limelightFront.CALCULATESHOOTVELO() : m_limelightBack.CALCULATESHOOTVELO())
        );
      },
      () -> {
        m_hopper.stopRoller();
        m_chute.setChuteSpeed(0);
        m_launcher.stop();
      },
      m_hopper, m_chute, m_launcher, m_limelightFront, m_limelightBack
    ));

    //Copilot Hopper Forward
    m_copilotController.rightTrigger(0.2)
    .whileTrue(new RunCommand(() -> m_hopper.setRollerVelocity(-5500), m_hopper))
    .onFalse(new InstantCommand(() -> m_hopper.stopRoller(), m_hopper));

    //Copilot Hopper Reverse
    m_copilotController.rightBumper()
    .whileTrue(new RunCommand(() -> m_hopper.setRollerVelocity(5500), m_hopper))
    .onFalse(new InstantCommand(() -> m_hopper.stopRoller(), m_hopper));

    //Copilot Chute Forward
    m_copilotController.leftTrigger(0.2)
    .whileTrue(new RunCommand(() -> m_chute.setChuteSpeed(1), m_chute))
    .onFalse(new InstantCommand(() -> m_chute.setChuteSpeed(0), m_chute));

    //Copilot Full Launch System With Limelight
    /*m_copilotController.x().whileTrue(Commands.startEnd(
      () -> {
        m_hopper.setRollerVelocity(-5500);
        m_chute.setChuteSpeed(1);
        m_launcher.setVelocity(
          (m_limelightFront.getLLTV() && m_limelightBack.getLLTV()) ? (m_limelightFront.CALCULATESHOOTVELO() + m_limelightBack.CALCULATESHOOTVELO()) / 2
          : (m_limelightFront.getLLTV() ? m_limelightFront.CALCULATESHOOTVELO() : m_limelightBack.CALCULATESHOOTVELO())
        );
      },
      () -> {
        m_hopper.stopRoller();
        m_chute.setChuteSpeed(0);
        m_launcher.stop();
      },
      m_hopper, m_chute, m_launcher, m_limelightFront, m_limelightBack
    ));*/
    m_copilotController.x()
    .whileTrue(new RunCommand(() -> m_launcher.setVelocity(
      (m_limelightFront.getLLTV() && m_limelightBack.getLLTV()) ? (m_limelightFront.CALCULATESHOOTVELO() + m_limelightBack.CALCULATESHOOTVELO()) / 2
      : (m_limelightFront.getLLTV() ? m_limelightFront.CALCULATESHOOTVELO() : m_limelightBack.CALCULATESHOOTVELO())
    ), m_launcher))
    .onFalse(new InstantCommand(() -> m_launcher.stop(), m_launcher));

    //Copilot Full Launch System With Velocity
   /*  m_copilotController.povUp().whileTrue(Commands.startEnd(
      () -> {
        m_hopper.setRollerVelocity(-5500);
        m_chute.setChuteSpeed(1);
        m_launcher.setVelocityByVelo();
      },
      () -> {
        m_hopper.stopRoller();
        m_chute.setChuteSpeed(0);
        m_launcher.stop();
      },
      m_hopper, m_chute, m_launcher
    )); */
    m_copilotController.povUp()
    .whileTrue(new RunCommand(() -> m_launcher.setVelocityByVelo(), m_launcher))
    .onFalse(new InstantCommand(() -> m_launcher.stop(), m_launcher));

    //Copilot Launch Low Speed
    m_copilotController.a()
    .whileTrue(new RunCommand(() -> m_launcher.setVelocity(3500), m_launcher))
    .onFalse(new InstantCommand(() -> m_launcher.stop(), m_launcher));

    //Copilot Launch Medium Speed
    m_copilotController.b()
    .whileTrue(new RunCommand(() -> m_launcher.setVelocity(4500), m_launcher))
    .onFalse(new InstantCommand(() -> m_launcher.stop(), m_launcher));

    //Copilot Launch High Speed
    m_copilotController.y()
    .whileTrue(new RunCommand(() -> m_launcher.setVelocity(5500), m_launcher))
    .onFalse(new InstantCommand(() -> m_launcher.stop(), m_launcher));

    //Copilot Launch Speeds
    m_copilotController.povLeft().whileTrue(new RunCommand(() -> {m_launcher.increaseVelo();}));
    m_copilotController.povRight().whileTrue(new RunCommand(() -> {m_launcher.decreaseVelo();}));
  }  

  public Command getAutonomousCommand() {
    //try{
      //return autoChooser.getSelected();  
      //SlideInCommand autoslidein = new SlideInCommand(m_fuelgrabber);
      //FullOuttakeOnVeloCommand autoshoot = new FullOuttakeOnVeloCommand(m_fuelgrabber, m_chute, m_launcher,
      //-4500, 1, 4000);
      auto1 auto = new auto1(m_intakeModule, m_hopper, m_chute, m_launcher);
      return auto;
    //} catch (Exception e) {
    //  return null;
    //}
  }
}