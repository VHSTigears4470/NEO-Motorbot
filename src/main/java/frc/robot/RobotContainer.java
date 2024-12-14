// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveSubsystem;

//private final SendableChooser autoChooser;

public class RobotContainer {
  
  private final SendableChooser<Command> autoChooser;
  private CommandXboxController xbox;
  private DriveSubsystem driveSubsystem;

  public RobotContainer() {
    this.xbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    this.driveSubsystem = new DriveSubsystem(xbox);
    driveSubsystem.setLeftInversion(false);
    driveSubsystem.setRightInversion(true);
    driveSubsystem.resetEncoders();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    
    autoChooser.addOption("NEO Test 1", new PathPlannerAuto("NEO Test 1"));
    // Configure the trigger bindings
    //configureBindings();
    initializeDriveMode(xbox);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  public void initializeDriveMode(CommandXboxController xbox) {
    if (DriveConstants.USING_DRIVE) {
      driveSubsystem.setDefaultCommand(new ArcadeDrive(driveSubsystem, xbox));
    } else {
      driveSubsystem.resetEncoders();
      driveSubsystem = null;
    }
  }

  

  /*  
  
    private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
  
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }
  */
  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return driveSubsystem.getAutonomousCommand("NEO Test 1", true);
  }

  public void reset(){
    driveSubsystem.resetEncoders();
  }
  
}
