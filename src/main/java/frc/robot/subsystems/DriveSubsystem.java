// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  CANSparkMax leftFrontMotor;
  CANSparkMax leftBackMotor;
  CANSparkMax rightFrontMotor;
  CANSparkMax rightBackMotor;

  RelativeEncoder lfEncoder;
  RelativeEncoder lbEncoder;
  RelativeEncoder rfEncoder;
  RelativeEncoder rbEncoder;

  AHRS gyro;
  DifferentialDriveOdometry odometer;
  DifferentialDrive differentialDrive;
  DifferentialDriveWheelPositions wheelPositions;

  public DriveSubsystem() {

    leftFrontMotor = new CANSparkMax(DriveConstants.leftFrontChannel, MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(DriveConstants.leftBackChannel, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(DriveConstants.rightFrontChannel, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(DriveConstants.rightBackChannel, MotorType.kBrushless);
    setInversions(true, false); // ORBB: false | AR: true

    lfEncoder = leftFrontMotor.getEncoder();
    lfEncoder.setPositionConversionFactor(DriveConstants.wheelRate); 
    lbEncoder = leftBackMotor.getEncoder();
    lbEncoder.setPositionConversionFactor(DriveConstants.wheelRate); 
    rfEncoder = rightFrontMotor.getEncoder();
    rfEncoder.setPositionConversionFactor(DriveConstants.wheelRate);
    rbEncoder = rightBackMotor.getEncoder();
    rbEncoder.setPositionConversionFactor(DriveConstants.wheelRate);
    resetEncoders();

    wheelPositions = new DifferentialDriveWheelPositions(lfEncoder.getPosition(), rfEncoder.getPosition());

    gyro = new AHRS(SPI.Port.kMXP); 
    odometer = new DifferentialDriveOdometry(gyro.getRotation2d(), lfEncoder.getPosition(), rfEncoder.getPosition(), 
                                             new Pose2d(0, 0, new Rotation2d())); //MODIFY (5, 13.5)
    
    differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    ReplanningConfig config = new ReplanningConfig(true, true, 0.1, 0.1); //TODO: UPDATE  

    AutoBuilder.configureRamsete(
            this::getPose, 
            this::resetPose, 
            this::getRobotRelativeSpeeds, 
            this::setChassisSpeeds,        
            // (ChassisSpeeds) -> odometer, 
            // new PPLTVController(0.02), 
            config, 
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this 
    );
  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd * 0.7, rot * 0.7);
  }

  public void stopMotors() {
    differentialDrive.arcadeDrive(0, 0);
  }

  public void resetEncoders() {
    lfEncoder.setPosition(0);
    lbEncoder.setPosition(0);
    rfEncoder.setPosition(0);
    rbEncoder.setPosition(0);
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometer.resetPosition(gyro.getRotation2d(), wheelPositions, pose);
  }

  public RelativeEncoder getLFncoder() {
    return lfEncoder;
  }

  public RelativeEncoder getLBncoder() {
    return lbEncoder;
  }

  public RelativeEncoder getRFncoder() {
    return rfEncoder;
  }

  public RelativeEncoder getLRBncoder() {
    return rbEncoder;
  }

  public double encoderAverage() {
    return (lfEncoder.getPosition() + lbEncoder.getPosition() + rfEncoder.getPosition() + rbEncoder.getPosition()) / 4;
  }

  public void resetPosition(Pose2d newPose){
    odometer.resetPosition(gyro.getRotation2d(), wheelPositions, newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(getDifferentialDriveWheelSpeeds());
  }

  public DifferentialDriveWheelSpeeds getDifferentialDriveWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(lbEncoder.getVelocity(), rbEncoder.getVelocity());
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    arcadeDrive(Math.signum(chassisSpeeds.vxMetersPerSecond) * Math.min(Math.abs(chassisSpeeds.vxMetersPerSecond / 3), 0.8), 
                Math.signum(chassisSpeeds.omegaRadiansPerSecond) * Math.min(Math.abs(chassisSpeeds.omegaRadiansPerSecond / 3), 0.8));
  }

  public void setInversions(Boolean invertedR, Boolean invertedL){
    leftBackMotor.follow(leftFrontMotor);
    leftFrontMotor.setInverted(invertedL);
    rightBackMotor.follow(leftFrontMotor);
    rightFrontMotor.setInverted(invertedR);
  }

  public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
    // Load the path you want to follow using its name in the GUI
    try {
      System.out.println("Trying");
      PathPlannerAuto path = new PathPlannerAuto(pathName);
      gyro.reset();
      System.out.println("STARTING POSE " + PathPlannerAuto.getStaringPoseFromAutoFile(pathName).toString());
      resetPose(PathPlannerAuto.getStaringPoseFromAutoFile(pathName));
      System.out.println("CURRENT POSE : " + odometer.getPoseMeters().getTranslation().toString());

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      return path;

    } catch (Exception e) {
      System.out.println(e);
      return null;
    }
  }

  @Override
  public void periodic() {
    Rotation2d gyroAngle = gyro.getRotation2d();  
    odometer.update(gyroAngle, (lfEncoder.getPosition() + lbEncoder.getPosition()) / 2, 
                   (rfEncoder.getPosition() + rbEncoder.getPosition()) / -2); //Updating pose

    SmartDashboard.putNumber("LF encoder", lfEncoder.getPosition());
    SmartDashboard.putNumber("RF encoder", rfEncoder.getPosition());
    SmartDashboard.putNumber("LB encoder", lbEncoder.getPosition());
    SmartDashboard.putNumber("RB encoder", rbEncoder.getPosition());
    SmartDashboard.putNumber("Encoder Avg", encoderAverage());
    SmartDashboard.putString("Pose2d", odometer.getPoseMeters().getTranslation().toString());
    SmartDashboard.putNumber("L Vel", lfEncoder.getVelocity());
    SmartDashboard.putNumber("R Vel", rfEncoder.getVelocity());    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
