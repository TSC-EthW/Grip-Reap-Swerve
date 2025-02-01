// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MotorTest extends Command {
  TalonFX motor = new TalonFX(15);
  private VoltageOut m_request = new VoltageOut(0);
  Supplier<Pose2d> Pose;
  /** Creates a new MotorTest. */
  public MotorTest(Supplier<Pose2d> poseXVal) {
    Pose = poseXVal;
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motor.setControl(m_request.withOutput(1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(Pose.get().getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.setControl(m_request.withOutput(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Pose.get().getX() >= 2;
  }
}
