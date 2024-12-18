// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final VictorSP intakeMotor = new VictorSP(IntakeConstants.intakeMotorChannel);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public Command runIntake(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setIntakeSpeed(speed);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
