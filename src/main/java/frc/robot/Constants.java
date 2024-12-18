// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class DrivetrainConstants {
    // PWM
    public static final int leftLeaderChannel = 1;
    public static final int leftFollowerChannel = 2;
    public static final int rightLeaderChannel = 3;
    public static final int rightFollowerChannel = 4;

    // Digital
    public static final int[] leftEncoderChannels = { 0, 1 };
    public static final int[] rightEncoderChannels = { 2, 3 };

    // Constants
    public static final double kTrackWidth = 0.381 * 2; // meters
    public static final double kWheelRadius = 0.0508; // meters
    public static final int kEncoderResolution = 4096;
  }

  public static class ChuteConstants {
    // PWM
    public static final int fanMotorChannel_1 = 5;
    public static final int fanMotorChannel_2 = 6;
    public static final int gateServoChannel = 7;

    // Digital
    public static final int lowerBreakBeamChannel = 4;
    public static final int upperBreakBeamChannel = 5;

    // Constants
    public static final double openAngle = 90;
    public static final double closeAngle = 0; 
  }

  public static class LifterConstants {
    // CAN (not PWM)
    public static final int lifterMotorChannel = 5;

    // Digital
    public static final int lifterLimitChannel = 6;

    // Analog
    public static final int toteDistanceSensorPort = 0;

    // Constants
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kSVolts = 1.0;
    public static final double kVVoltSecondsPerRotation = 1.0;
    public static final double LifterTolerance = 10.0;
    public static final double kEncoderDistancePerPulse = 1.0;
    public static final double kLifterTarget = 0;
    // Position States for the Lifter Arm.
    public static final double kHoldPosition    = 10;
    public static final double kPrepPosition    = 20;
    public static final double kCapturePosition = 30;

  }

  public static class VisionConstants {}

  public static class IntakeConstants {
    // PWM
    public static final int intakeMotorChannel = 8;
  }
}
