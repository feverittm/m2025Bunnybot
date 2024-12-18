// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LifterConstants;

import static frc.robot.Constants.LifterConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * The Totemover is a 4-bar linkage attached to the rear of the robot and
 * designed to extent
 * out the back and drop onto totes and then move them back into the robot.
 * 
 * This subsystem is designed to move the linkage to a specific position by
 * moving a motor to a specific angle.
 * The linkage is powered by a Neo motor on a high-reduction gearbox (100:1).
 * The final gear will not even need
 * to make a full revolution in order for the linkage to move from fully
 * retracted to fully extended.
 * 
 * It is assumed that the encoder measures positive with arm extension. 0->N,
 * where N is fully extended.
 * 
 * There will be 2 control modes: 1 - Manual control of position with limits, 2
 * - Defined positions specified
 * by arm state.
 * 
 * There are 4 distinct (defined) positions:
 * 1 - Stowed. Fully back inside the robot. This is the starting position. Also
 * this position activates
 * a limit switch and when active will reset the motor encoder.
 * 2 - Hold. Position for holding the tote in the robot. Normal inside position.
 * 3 - Prepare. This position is outside the robot bot not low enough to pick up
 * a tote. It is a preperation
 * position for when the robot moves in to pick up a tote.
 * 4 - Capture. This position is the fully extented position. It is expected
 * that this position will
 * drop the clips onto the tote.
 */

public class ToteMoverSubsystem extends SubsystemBase {
  private final SparkMax lifterMotor = new SparkMax(LifterConstants.lifterMotorChannel, MotorType.kBrushless);
  private SparkClosedLoopController closedLoopController = lifterMotor.getClosedLoopController();
  private final RelativeEncoder lifterEncoder = lifterMotor.getEncoder();

  public enum ArmPosition {
    MANUAL, STOWED, HOLD, PREPARE, CAPTURE
  }

  private double getArmPositionValue(ArmPosition position) {
    return switch (position) {
      case MANUAL -> lifterEncoder.getPosition();
      case STOWED -> 0;
      case HOLD -> 1;
      case PREPARE -> 2;
      case CAPTURE -> 3;
    };
  }

  public static enum MoveMode {
    MANUAL, AUTO
  }

  private static ArmPosition ArmState = ArmPosition.STOWED;

  private static Boolean isArmReset = false; 

  private static MoveMode MoveState = MoveMode.MANUAL;

  /** Creates a new ToteMoverSubsystem. */
  public ToteMoverSubsystem() {
    ConfigureMotor();

    zeroPosition();

    // setSetpoint(LifterConstants.kLifterTarget);
  }

  private void ConfigureMotor() {
    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAXes that we will set below.
     */
    SparkMaxConfig tmConfig = new SparkMaxConfig();

    /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */
    tmConfig
        .smartCurrentLimit(50)
        .inverted(false)
        .idleMode(IdleMode.kBrake);

    tmConfig.encoder
        .positionConversionFactor(LifterConstants.kEncoderDistancePerPulse)
        .velocityConversionFactor(LifterConstants.kEncoderDistancePerPulse);

    tmConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.0, 0, 0)
        .outputRange(-1, 1)
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0.0, ClosedLoopSlot.kSlot1)
        .d(0.0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    tmConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);



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
    lifterMotor.configure(tmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);

  }

  private void invertMotor() {
    boolean isInverted = lifterMotor.configAccessor.getInverted();
  
    SparkMaxConfig invertConfig = new SparkMaxConfig();
    invertConfig.inverted(true);

    SparkMaxConfig noinvertConfig = new SparkMaxConfig();
    invertConfig.inverted(false);

    if (isInverted) {
      lifterMotor.configure(noinvertConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    } else {
      lifterMotor.configure(invertConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

  }

  public void setMotorVoltage(double voltage) {
    if (getMeasurement() == 0) {
      System.out.println("Monual: Arm Lower Limit Reached");
      voltage = 0;
    } else if (getMeasurement() >= LifterConstants.kCapturePosition) {
      System.out.println("Manual: Arm Extension Limit Reached");
      voltage = 0;
    }
    lifterMotor.setVoltage(voltage);
  }

  /**
   * Move the arm to position with limits.
   * 
   * @param output   Voltage to be applied to motors
   * @param setpoint Position setpoint
   */
  public void useOutput(double output, double setpoint) {
    if (getMeasurement() == setpoint) {
      return;
    } else if (getMeasurement() == 0) {
      System.out.println("PID: Arm Lower Limit Reached");
      setpoint = 0;
      return;
    } else if (getMeasurement() >= LifterConstants.kCapturePosition) {
      System.out.println("PID: Arm Extension Limit Reached");
      setpoint = LifterConstants.kCapturePosition;
      return;
    } else {
      MoveState = MoveMode.AUTO;
      return;
    }
  }

  /**
   * Move arm to position given by the mode bit. This is NOT a move to a given
   * position (as in an actual encoder position) but a programmed position or
   * just move manually. Everything is held to limits.
   * We will use the 'hat' switch to control the movement:
   * Up: Manually Extend the arm and save current position. (hat = 0)
   * Right: Move arm to next extended programmed PID position. (hat = 90)
   * Down: Manually retract the arm and save the position. (hat = 180)
   * Left: Move to the previous programmmed position. (hat = 270)
   * 
   * @param mode      Are we used the presets or moving manually?
   * @param direction Extend (True), or Retract (False)
   * 
   *                  Expected Bahaviour:
   *                  - If we are in manual mode then stop the arm and grab the
   *                  current position value.
   *                  them try to determine the closest define position to that
   *                  angle. This was the
   *                  Previous/Next function will work correctly.
   *                  - Pushing the left or right button on the hat switch will
   *                  mode the arm
   *                  to the previous or next position in our list.
   *                  - Manual mode is where the fun is...
   *                  Default action (and action when button is released) is to
   *                  stop the arm
   *                  in it's current position.
   *                  Move action will save the current position, manually (by
   *                  directly setting the
   *                  voltage to the motor) move the arm as long as we have not
   *                  reached the limits.
   * 
   */
  public void moveArmToPosition(MoveMode mode, Boolean direction) {
    if (mode == MoveMode.AUTO) {
      if (MoveState == MoveMode.MANUAL) {
        ArmState = getClosestPosition(direction);
        moveToArmSetpoint(ArmState);
      }
    } else { // MoveMode.MANUAL
      ArmState = ArmPosition.MANUAL;
      setMotorVoltage(direction ? 0.5 : -0.5);
    }
    MoveState = mode;
  }

  public Command moveToArmSetpoint(ArmPosition position) {
    MoveState = MoveMode.AUTO;
    ArmState = position;
    return this.runOnce(() -> 
      closedLoopController.setReference(getArmPositionValue(position), ControlType.kMAXMotionPositionControl));
  }

  public void stopArmManual(MoveMode mode) {
    if (mode == MoveMode.MANUAL) {
      setMotorVoltage(0);
    }
  }

  /**
   * Find the closest defined position to the current raw arm position.
   * 
   * @return the arm position
   */
  private ArmPosition getClosestPosition(Boolean direction) {
    double currentPosition = getMeasurement();
    for (ArmPosition n : ArmPosition.values()) {
      double armSetpoint = getArmPositionValue(n);
      if (direction = true) {
        if (armSetpoint >= currentPosition) {
          return n;
        }
      } else {
        // TODO: reverse search. Need to return the previous setpoint.
        if (armSetpoint <= currentPosition) {
          return n;
        }
      }
    }
    return ArmPosition.MANUAL;
  }

  /**
   * Current used for the lifter
   * 
   * @return Current being used by the lifter.
   */
  public double getLifterCurrent() {
    return lifterMotor.getOutputCurrent();
  }

  /**
   * should be a trigger to reset encoder.
   */
  public void zeroPosition() {
    if (getMeasurement() == 0) {
      System.out.println("Reset Arm Encoder Position");
      ArmState = ArmPosition.STOWED;
      //lifterEncoder.setPosition(0);
    }
  }

  /**
   * Use this to get arm position
   */
  public double getMeasurement() {
    return lifterEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Lifter Arm State", ArmState.name());
    SmartDashboard.putString("Lifter Move State", MoveState.name());
    SmartDashboard.putNumber("Lifter Current", getLifterCurrent());
    SmartDashboard.putNumber("Lifter Position", getMeasurement());
  }

  /** Manually Extent the tote mover. */
  public Command toteMoverManualExtend() {
    // implicitly require `this`
    return this.runOnce(() -> moveArmToPosition(MoveMode.MANUAL, true));
  }

  /** Manually Extent the tote mover. */
  public Command toteMoverManualRetract() {
    // implicitly require `this`
    return this.runOnce(() -> moveArmToPosition(MoveMode.MANUAL, false));
  }

  /** Manually Extent the tote mover. */
  public Command toteMoverAutoPrev() {
    return this.runOnce(() -> moveArmToPosition(MoveMode.AUTO, true));
  }

  /** Manually Extent the tote mover. */
  public Command toteMoverAutoNext() {
    return this.runOnce(() -> moveArmToPosition(MoveMode.AUTO, false));
  }

  /** Stop the tote mover. */
  public Command stoptoteMover() {
    return this.runOnce(() -> stopArmManual(MoveState));
  }
}
