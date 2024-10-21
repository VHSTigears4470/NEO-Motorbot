// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  CommandXboxController xboxController;

  public DriveSubsystem(CommandXboxController xboxController) {

    leftFrontMotor = new CANSparkMax(DriveConstants.leftFrontChannel, MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(DriveConstants.leftBackChannel, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(DriveConstants.rightFrontChannel, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(DriveConstants.rightBackChannel, MotorType.kBrushless);

    lfEncoder = leftFrontMotor.getEncoder();
    lbEncoder = leftBackMotor.getEncoder();
    rfEncoder = rightFrontMotor.getEncoder();
    rbEncoder = rightBackMotor.getEncoder();

    wheelPositions = new DifferentialDriveWheelPositions(lfEncoder.getPosition(), rfEncoder.getPosition());

    gyro = new AHRS(SPI.Port.kMXP); 
    odometer = new DifferentialDriveOdometry(gyro.getRotation2d(), lfEncoder.getPosition(), rfEncoder.getPosition(), 
                                             new Pose2d(5.0, 13.5, new Rotation2d())); //ADD POSE2D
    
    lfEncoder.setPositionConversionFactor(1); // TODO: Find relative to wheels instead of motors
    lbEncoder.setPositionConversionFactor(1);
    rfEncoder.setPositionConversionFactor(1);
    rbEncoder.setPositionConversionFactor(1);

    leftBackMotor.follow(leftFrontMotor);
    leftFrontMotor.setInverted(false);
    rightBackMotor.follow(rightFrontMotor);
    rightFrontMotor.setInverted(true); // ORBB: false | AR: true

    differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    this.xboxController = xboxController;
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

  @Override
  public void periodic() {
    var gyroAngle = gyro.getRotation2d();
    // This method will be called once per scheduler run
    Pose2d pose = odometer.update(gyroAngle, lfEncoder.getPosition(), rfEncoder.getPosition()); //Updating pose
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
