// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * A skeleton for a command to pick up a tote.  I don't know if I will use the vision 
 * to align and approach the tote (probably will make it a separate command).
 * 1 - Extend tote lifter to the 'Prepare' Height.
 * 2 - Align to Tote
 * 3 - move forward until tote hits robot (use ultrasonic rangefinder to approach specific distance)
 * 4 - Fully extend lifter to 'Capture' Height
 * 5 - Retract lifter to 'HOLD' position
 * 6 - Verify capture.
 * 
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ToteMoverSubsystem;

public class GrabTote extends Command {
  private final ToteMoverSubsystem m_toteMover;
  /** Creates a new GrabTote. */
  public GrabTote(ToteMoverSubsystem mover) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_toteMover = mover;
    
    addRequirements(m_toteMover);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
