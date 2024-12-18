// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.APDS9960;
import static frc.robot.Constants.ChuteConstants;

public class ChuteSubsystem extends SubsystemBase {
  /** Creates a new ChuteSubsystem. */
  private static VictorSP fanMotor_1 = new VictorSP(ChuteConstants.fanMotorChannel_1);
  private static VictorSP fanMotor_2 = new VictorSP(ChuteConstants.fanMotorChannel_2);

  private static Servo gateServo = new Servo(ChuteConstants.gateServoChannel);

  private static DigitalInput lowerBreakBeam = new DigitalInput(ChuteConstants.lowerBreakBeamChannel);
  private static DigitalInput upperBreakBeam = new DigitalInput(ChuteConstants.upperBreakBeamChannel);

  private APDS9960 apds = new APDS9960();

  public int balloonCount = 0;

  public ChuteSubsystem() {
    fanMotor_1.addFollower(fanMotor_2);
  }

  public void openGate() {
    double currentAngle = gateServo.getAngle();
    SmartDashboard.putNumber("Gate Angle", currentAngle);
    gateServo.setAngle(0);
  }

  public void setMotorSpeed(double speed) {
    fanMotor_1.set(speed);
  }

  public boolean getLowerBreakBeam() {
    return lowerBreakBeam.get();
  }

  public double[] getColorArray() {
    Color color = apds.readColor();
    double[] colors = {color.red, color.green, color.blue};
    return colors;
  }

  public Command runFanCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setMotorSpeed(1.0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumberArray("Color Sensor", getColorArray());
  }
}
