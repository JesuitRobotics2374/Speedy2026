// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
  
  private TalonFX intakeMotor;
  private boolean intaking;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(32, "rio");
    intaking = false;
  }

  /**
   * Stops intaking
   */
  private void stopIntake() {
    intaking = false;
    intakeMotor.stopMotor();
  }

  /**
   * Stops intaking and reverse intake to get rid of a stuck fuel or smth
   * @param speed the speed that the intake purges the fuel
   */
  private void purge(double speed) {
    intaking = false;
    intakeMotor.set(-speed);
  }

  /**
   * Intakes fuel into the hopper
   * @param speed the speed that the robot intakes
   */
  private void intakeFuel(double speed) {
    intaking = true;
    intakeMotor.set(speed);
  }

  /**
   * Tells if intake is intaking
   * @return true = intaking
   */
  public boolean getIntaking() {
    return intaking;
  }

  public Command intake() {
    return new InstantCommand(()-> intakeFuel(1));
  }

  public Command purge() {
     return new InstantCommand(()-> purge(1));
    // key binds for purging (reversing intake motors) intake
  }

  public Command stop() {
    return new InstantCommand(()-> stopIntake());
    // keybinds for stopping intake motors
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}