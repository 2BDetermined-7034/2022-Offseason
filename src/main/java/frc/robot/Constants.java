// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveBase {
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5779;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5779;
    }
    public static final class Modules {
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(266.6); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(68.7); // FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 14;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(160.83); // FIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(265.5); // FIXME Measure and set back right steer offset
    }

    public static final class Subsystem {
        public static final class Intake {
            public static final int intakeMotor1 = 0; //FIXME
            public static final int intakeMotor2 = 0; //FIXME

            public static final int solenoidForward = 3; //FIXME
            public static final int solenoidReverse = 2; //FIXME
            public static final double stopSpeed = 0.35; //speed when wrong color
            public static final int IDcompressor = 0; //fix maybe?
        }
        public static final class Indexer {
            public static final int lowerIndexer = 0; //FIXME
            public static final int higherIndexer = 0; //FIXME
            public static final double speed = 0.85;
        }

        public static final class Shooter {
            public static final int leftShooterMotor = 0; //FIXME
            public static final int rightShooterMotor = 0; //FIXME
            public static final int passiveOneSpeed = 0;
            public static final int passiveFullSpeed = 0;
            public static final double shooterInc = 0.05;
            public static final double shooterRange = 0.005; //change?

        }

        public static final class Climber {
            public static final int talonFXMotor = 0; //FIXME
            public static final int solenoidForwardID = 0; //FIXME
            public static final int solenoidBackID = 0; //FIXME
            public static final int maxPos = 227440;
            public static final int extendedValue = 312252;
            public static final int encoderAcceptableError = 10;
        }
        public static final class Vision {
            public static final int pGain = 16;

            public static final double VisX_kP = 1;
            public static final double VisX_MAX = .3;
            public static final double VisTimeLimit = 5;
            public static final double VisX_Offset = 0;
            public static final double VisX_Tol = 0;
            public static final double VisY_Tol = 5;
            public static final double VisY_distanceConstant = 254.3;
            public static final double VisY_Offset = 8;
            public static final double VisY_VTol = 100;
            public static final double VisX_VTol = 100;
            public static final int Vis_TimerConfidence = 5;
            public static double Vis_LLAngle = 35;
        }
    }
    public static final class pneumatics {
        public static final PneumaticsModuleType compressor = PneumaticsModuleType.REVPH;
        public static final PneumaticsModuleType shifter = PneumaticsModuleType.REVPH;
        public static final PneumaticsModuleType intake = PneumaticsModuleType.REVPH;
        public static final PneumaticsModuleType climber = PneumaticsModuleType.REVPH;
    }
}
