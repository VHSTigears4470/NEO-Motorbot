// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AutoConstants {
    // RobotConfig config;
    // try{
    //   config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    //   e.printStackTrace();
    // }
  }

  public final class DriveConstants {
    public static final boolean USING_DRIVE = true;

    //Update All Channels If Needed
    //public static final int leftFrontChannel = 2; 
    //public static final int leftBackChannel = 1; 
    //public static final int rightFrontChannel = 4; 
    //public static final int rightBackChannel = 3; 

    // Update All Channels If Needed
    public static final int leftFrontChannel = 2; // One-Red-Bumper-Bot (ORBB): 3 | All-Red: 0
    public static final int leftBackChannel = 1; // ORBB: 2 | AR: 2
    public static final int rightFrontChannel = 4; // ORBB: 0 | AR: 1
    public static final int rightBackChannel = 3; // ORBB: 1 | AR: 3

    public static final double wheelRate = 6 * Math.PI / 10.71; //1/10.71, 1/8.45, 1/5.95


    public static final DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(Units.inchesToMeters(1.0)); //UPDATE TRACK WIDTH
    public static ChassisSpeeds chassisSpeeds = 
        new ChassisSpeeds(2.0, 0, 1.0); //UPDATE VELOCITIES
    public static final DifferentialDriveWheelSpeeds wheelSpeeds =
        kinematics.toWheelSpeeds(chassisSpeeds); //UPDTE WHEEL SPEEDS
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;
    double linearVelocity = chassisSpeeds.vxMetersPerSecond; //Linear Velocity
    double angularVelocity = chassisSpeeds.omegaRadiansPerSecond; //Angular Velocity  
  }
}
