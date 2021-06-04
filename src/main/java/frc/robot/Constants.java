// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class PhysicalRobotConstants {
        public static final double kTrackWidthMeters = 0.552976;
        //change these values for yith
        public static final double kS = 0.579; //Volts
        public static final double kV = 2.37; //VoltsPerMeter
        public static final double kA = 0.21; //VoltsPerMeterSquared
        public static final double feetPerTick = 1.2207031E-4; //feet conversion from ticks(one cycle on talon)
        public static final double metersPerTick = (2048/4.67) * (1000/Math.PI * 6 * 2.54); //don't use please
        public static final double kMaxVoltage = 8; //max is 12V
        public static final double tickSpeedInMetersPerSec = 1861.2; //def wrong, don't use
        public static final int kFalconCPR = 4096; // maybe 4096 but go with 2048 but also go with 4096
        public static final double kGearRatio = 4.67;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double ticksPerRotation = kFalconCPR / kGearRatio;
        final int kCountsPerRev = 4096;  //Encoder counts per revolution of the motor shaft.
        final double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
        final double kGearRatioPT = 10.71; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
        final double kWheelRadiusInches = 3;
        final int k100msPerSecond = 10;
    }

    public static final class DriveTalonIDs {
        public static final int leftFrontID = 3;
        public static final int leftBackID = 2;
        public static final int rightFrontID = 0;
        public static final int rightBackID = 1;
    }

    public static final class DriveConstants {
        //change these values for yith
        public static final double kP = .0221;//0.00048; //2.21 is for WPILib PID integration, use 0.00048 for TalonFX PID integration
        public static final double kI = 0.002;
        public static final double kD = 0.02;
        public static final double maxV = 3; //MetersPerSecond
        public static final double maxA = 1.5; //MetersPerSecondSquared
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(PhysicalRobotConstants.kTrackWidthMeters);
        

    }
}