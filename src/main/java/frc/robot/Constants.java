// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {


//Intake PID and Encoder Constants
    public static class IntakeConstants {
        public static final double kIntakeDroppedAngle = 9.0;
        public static final double kIntakeRaisedAngle = 9.0;
        public static final int kIntakeMotorID = 0;
        public static final int kArmMotorID = 0;
        public static final double kIntakeP = 0;
        public static final double kIntakeI = 0;
        public static final double kIntakeD = 0;
        public static final double kArmP = 0;
        public static final double kArmI = 0;
        public static final double kArmD = 0;
        public static final int kArmEncoderCh = 0;
    }

    //Shooter subsystem speed constants
    public static class ShooterConstants {
        public static final double kSpinSpeedTrue = 0.75;
        public static final double kSpinSpeedFalse = 0;


    }
}

