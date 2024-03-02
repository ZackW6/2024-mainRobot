// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class Candle extends SubsystemBase {
    CANdle candle1 = new CANdle(1, Constants.mainCanbus);
    CANdle candle2 = new CANdle(2, Constants.mainCanbus);

    // CANdle candle2 = new CANdle(31, Constants.mainCanbus);

private final int LedCount = 86;

private Animation toAnimate;
  /** Creates a new CANdleSubsystem. */
  public Candle() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.3;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    
    candle1.configAllSettings(configAll, 100);
    candle2.configAllSettings(configAll, 100);

    toAnimate = new TwinkleOffAnimation(157, 3, 252, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
  }

  @Override
  public void periodic() {
    candle1.animate(toAnimate);
    candle2.animate(toAnimate);
  }

  public Command shootingLightsFlash(double speed) {
    return this.runOnce(() -> toAnimate = new StrobeAnimation(157, 3, 252, 0, speed, LedCount));
  }

  public Command idleLED() {
    return this.runOnce(() -> toAnimate = new TwinkleOffAnimation(157, 3, 252, 0, 0.8, LedCount, TwinkleOffPercent.Percent100));
  }

  public Command rushHourLights() {
    return this.runOnce(() -> toAnimate = new RainbowAnimation(1, 1, LedCount));
  }

  public Command intake() {
    return this.runOnce(() -> toAnimate = new ColorFlowAnimation(255, 255, 0, 0, 0.7, LedCount, Direction.Forward));
  }

  public Command pickUpLights() {
    return this.runOnce(() ->{
        toAnimate = new StrobeAnimation(0, 255, 0, 0, 0.05, LedCount);
    } );
  }

  public Command warningLights() {
    return this.runOnce(() -> toAnimate = new StrobeAnimation(255, 0, 0, 0, 0.8, LedCount));
  }
}