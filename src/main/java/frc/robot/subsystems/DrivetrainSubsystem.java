// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  public final double kMaxSpeed = 3.0; // meters per second
  public final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  private final SparkMax m_leftLeader =    new SparkMax(DrivetrainConstants.leftLeaderChannel, MotorType.kBrushless);
  private final SparkMax m_leftFollower =  new SparkMax(DrivetrainConstants.leftFollowerChannel, MotorType.kBrushless);
  private final SparkMax m_rightLeader =   new SparkMax(DrivetrainConstants.rightLeaderChannel, MotorType.kBrushless);
  private final SparkMax m_rightFollower = new SparkMax(DrivetrainConstants.rightFollowerChannel, MotorType.kBrushless);

  private final Encoder m_leftEncoder = new Encoder(DrivetrainConstants.leftEncoderChannels[0],
      DrivetrainConstants.leftEncoderChannels[1]);
  private final Encoder m_rightEncoder = new Encoder(DrivetrainConstants.rightEncoderChannels[0],
      DrivetrainConstants.rightEncoderChannels[1]);

  private AHRS ahrs;

  private boolean m_driveDirection = true;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    ConfigureDriveMotors();

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_driveDirection = true;
  }

  private void ConfigureDriveMotors() {
    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAXes that we will set below.
     */
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */
    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    // Apply the global config and invert since it is on the opposite side
    rightLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    // Apply the global config and set the leader SPARK for follower mode
    leftFollowerConfig
        .apply(globalConfig)
        .follow(m_leftLeader);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig
        .apply(globalConfig)
        .follow(m_rightLeader);

    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    m_leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   * Set driving direction
   * 
   * @param boolean direction
   *                true: Regular forward with intake/chute forward.
   *                false: Reversed. Totermover is set as the 'front' of the
   *                robot.
   */
  public void setDriveDirection(boolean direction) {
    m_driveDirection = direction;
    SmartDashboard.putBoolean("Drive Direction", m_driveDirection);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param forward Linear velocity i.
   * @param rotation
   */
  public void drive(double forward, double rotation) {
    m_leftLeader.set(forward + rotation);
    m_rightLeader.set(forward - rotation);
  }

  public Command setDriveForward() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setDriveDirection(true);
        });
  }

  public Command setDriveReverse() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setDriveDirection(false);
          ;
        });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle", ahrs.getAngle());
  }
}
