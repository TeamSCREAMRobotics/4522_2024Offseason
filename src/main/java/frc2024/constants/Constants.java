// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class Constants {
  public static final double GRAVITY = 9.80665;

  public static final double PERIOD_SEC = 0.02; // 20 ms
  public static final double PERIOD_HZ = 1 / PERIOD_SEC; // 50 hz
  public static final double SIM_PERIOD_SEC = 0.001; // 1 ms

  public static final Mode ROBOT_MODE = Mode.SIM;

  public static enum Mode{
    REAL, SIM, REPLAY;
  }
}
