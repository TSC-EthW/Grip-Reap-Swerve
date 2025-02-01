// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autos extends SubsystemBase {
  public SendableChooser<Command> auto;
  /** Creates a new Autos. */
  public Autos() {
    auto = new SendableChooser<Command>();
    auto.setDefaultOption("Do Nothing", new PathPlannerAuto("Do Nothing"));
    auto.addOption("Bread&Butter", new PathPlannerAuto("Bread And Butter"));

    SmartDashboard.putData(auto);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
