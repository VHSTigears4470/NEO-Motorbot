// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class DriveConstants {
    public static final boolean USING_DRIVE = true;
    //Update All Channels If Needed
    public static final int leftFrontChannel = 0; // One-Red-Bumper-Bot (ORBB): 3 | All-Red: 0
    public static final int leftBackChannel = 2; // ORBB: 2 | AR: 2
    public static final int rightFrontChannel = 1; // ORBB: 0 | AR: 1
    public static final int rightBackChannel = 3; // ORBB: 1 | AR: 3
  }
}