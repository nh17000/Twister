// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double LOOP_PERIOD = 0.02; // 20ms
    public static final double LOOP_FREQUENCY = 1.0 / LOOP_PERIOD; // 50Hz
    public static final double NOMINAL_VOLTAGE = 12;
    public static final double g = 11; // 9.79285 m/s^2 in Houston

    public static final class AlignConstants {
        public static final double DRIVE_kP = 5.0; // m/s per m error
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;
        public static final double MAX_DRIVE_VELOCITY = 3.0; // m/s
        public static final double MAX_DRIVE_ACCELERATION = 10; // m/s^2

        public static final double ROT_kP = 5.0; // rads per rad error
        public static final double ROT_kI = 0.0;
        public static final double ROT_kD = 0.0;
        public static final double MAX_ROT_VELOCITY = 8; // rad/s
        public static final double MAX_ROT_ACCELERATION = 20; // rad/s^2

        public static final double ALIGN_ROT_TOLERANCE = Units.degreesToRadians(3);
        public static final double ALIGN_TRANSLATION_TOLERANCE = Units.inchesToMeters(2);

        public static final Translation2d AMP_OFFSET =
                new Translation2d(Units.inchesToMeters(0), -Units.inchesToMeters(21));
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
        public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);

        public static final Pose2d BLUE_AMP = new Pose2d(Units.inchesToMeters(72.455), FIELD_WIDTH, Rotation2d.kZero);
        public static final Pose2d RED_AMP =
                new Pose2d(FIELD_LENGTH - Units.inchesToMeters(72.455), FIELD_WIDTH, Rotation2d.kZero);

        public static final List<Pose2d> AMPS = List.of(BLUE_AMP, RED_AMP);

        private static final Translation3d TOP_RIGHT_SPEAKER = new Translation3d(
                Units.inchesToMeters(18.055), Units.inchesToMeters(238.815), Units.inchesToMeters(83.091));

        private static final Translation3d BOTTOM_LEFT_SPEAKER =
                new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

        /** Center of the speaker opening (blue alliance) */
        public static final Translation3d BLUE_SPEAKER_CENTER = BOTTOM_LEFT_SPEAKER.interpolate(TOP_RIGHT_SPEAKER, 0.5);

        public static final Pose2d BLUE_SPEAKER_POSE =
                new Pose2d(BLUE_SPEAKER_CENTER.getX(), BLUE_SPEAKER_CENTER.getY(), Rotation2d.kZero);
        public static final Pose2d RED_SPEAKER_POSE =
                new Pose2d(FIELD_LENGTH - BLUE_SPEAKER_CENTER.getX(), BLUE_SPEAKER_CENTER.getY(), Rotation2d.kZero);

        public static final double SPEAKER_HEIGHT = BLUE_SPEAKER_CENTER.getZ();
    }

    public static final class VisualizerConstants {
        public static final Translation3d SHOOTER_ZERO = new Translation3d(0.0, 0.0, 0.333377);
        public static final Translation3d DIVERTER_ZERO = new Translation3d(-0.178816, 0.0, 0.470029);
    }

    public static final class TurretConstants {
        public static final TalonFXConfiguration getTurretConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 50;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 50;

            config.MotionMagic.MotionMagicCruiseVelocity = 20;
            config.MotionMagic.MotionMagicAcceleration = 75;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.67;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.05;

            return config;
        }

        public static final int TURRET_ID = 20;
        public static final double TURRET_GEAR_RATIO = 95. / 10.;
        public static final double TURRET_P_COEFFICIENT = 2 * Math.PI / TURRET_GEAR_RATIO;

        public static final double TURRET_STARTING_ANGLE = Units.degreesToRadians(0);
        public static final double TURRET_MIN_ANGLE = Units.degreesToRadians(-270);
        public static final double TURRET_MAX_ANGLE = Units.degreesToRadians(270);

        public static final double TURRET_MASS = Units.lbsToKilograms(16);
        public static final double TURRET_CG_RADIUS = Units.inchesToMeters(3.75);

        // mass ≈ 16 lb, Lzz ≈ 494 in^2 lb
        // center of mass of turret ≈ 3.75 in from its axis of rotation
        // I = I_cm + md^2 = 494 + 16(3.75)^2 = 719 in^2 lb ≈ 0.21 kg m^2
        public static final double TURRET_MOI = 0.21;

        public static final DCMotor TURRET_MOTOR = DCMotor.getKrakenX60(1);

        // feedforward term: adds a voltage to the turret as the chassis rotates
        public static final double K_OMEGA = 0.1; // volts per radian per second

        public static final double SAFETY_LIMIT = Units.degreesToRadians(5);
        public static final double TURRET_SAFE_MIN = TURRET_MIN_ANGLE + SAFETY_LIMIT;
        public static final double TURRET_SAFE_MAX = TURRET_MAX_ANGLE - SAFETY_LIMIT;

        // only apply feedforward if the turret is within 45 degrees of its setpoint
        public static final double FF_ERROR_THRESHOLD = Units.degreesToRadians(45);
        // only apply feedforward if the drivetrain is rotating at a reasonable speed
        public static final double FF_CHASSIS_ROT_VELOCITY_LIMIT = 1.5 * Math.PI; // rad/s

        public static final Rotation2d AMP_SETPOINT = Rotation2d.k180deg;
    }

    public static final class IntakeConstants {
        public enum IntakeState {
            OFF(0),
            INTAKING(6),
            EJECTING(-6);

            public final double rollerVolts;

            private IntakeState(double rollerVolts) {
                this.rollerVolts = rollerVolts;
            }
        }

        public static final TalonFXConfiguration getRollerConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 20;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static final int ROLLER_ID = 51;
        public static final double ROLLER_GEAR_RATIO = (35. / 14.); // 2.5
        public static final double ROLLER_P_COEFFICIENT = 2 * Math.PI / ROLLER_GEAR_RATIO;

        public static final double ROLLER_MASS = Units.lbsToKilograms(2);
        public static final double ROLLER_RADIUS = Units.inchesToMeters(1);
        // public static final double ROLLER_MOI = 1.0 / 2.0 * ROLLER_MASS * ROLLER_RADIUS * ROLLER_RADIUS;
        public static final double ROLLER_SHAFT_MOI = 0.001; // ROLLER_MOI * ROLLER_GEAR_RATIO * ROLLER_GEAR_RATIO;

        public static final DCMotor ROLLER_MOTOR = DCMotor.getKrakenX60(1);
    }

    public static final class DiverterConstants {
        public enum DiverterState {
            STOWED(PIVOT_MAX_ANGLE, 0),
            DEPLOYED(PIVOT_STARTING_ANGLE, 6),
            REVERSE(PIVOT_STARTING_ANGLE, -6);

            public final double pivotRads;
            public final double rollerVolts;

            private DiverterState(double pivotRads, double rollerVolts) {
                this.pivotRads = pivotRads;
                this.rollerVolts = rollerVolts;
            }
        }

        public static final TalonFXConfiguration getPivotConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 20;

            config.MotionMagic.MotionMagicCruiseVelocity = 20;
            config.MotionMagic.MotionMagicAcceleration = 75;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.67;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;

            return config;
        }

        public static final TalonFXConfiguration getRollerConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 20;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static final int PIVOT_ID = 30;
        public static final double PIVOT_GEAR_RATIO = (42. / 14.) * (66. / 20.) * (50. / 14.); // ~35.36
        public static final double PIVOT_P_COEFFICIENT = 2 * Math.PI / PIVOT_GEAR_RATIO;

        public static final int ROLLER_ID = 31;
        public static final double ROLLER_GEAR_RATIO = (35. / 14.); // 2.5
        public static final double ROLLER_P_COEFFICIENT = 2 * Math.PI / ROLLER_GEAR_RATIO;

        public static final double PIVOT_STARTING_ANGLE = Units.degreesToRadians(48.1);
        public static final double PIVOT_MIN_ANGLE = Units.degreesToRadians(-45);
        public static final double PIVOT_MAX_ANGLE = Units.degreesToRadians(158);

        public static final double PIVOT_MASS = Units.lbsToKilograms(7);
        public static final double PIVOT_LENGTH = Units.inchesToMeters(18);

        public static final double ROLLER_MASS = Units.lbsToKilograms(2);
        public static final double ROLLER_RADIUS = Units.inchesToMeters(1);
        public static final double ROLLER_MOI = 1.0 / 2.0 * ROLLER_MASS * ROLLER_RADIUS * ROLLER_RADIUS;
        public static final double ROLLER_SHAFT_MOI = ROLLER_MOI * ROLLER_GEAR_RATIO * ROLLER_GEAR_RATIO;

        public static final DCMotor PIVOT_MOTOR = DCMotor.getKrakenX60(1);
        public static final DCMotor ROLLER_MOTOR = DCMotor.getKrakenX60(1);
    }

    public static final class TransferConstants {
        public enum TransferState {
            OFF(0),
            TRANSFERRING(1.7),
            REVERSE(-3);

            public final double volts;

            private TransferState(double volts) {
                this.volts = volts;
            }
        }

        public static final TalonFXConfiguration getTransferConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 20;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static final int LEFT_ID = 23;
        public static final int RIGHT_ID = 24;

        public static final double TRANSFER_GEAR_RATIO = 24. / 18.; // 1.33
        public static final double TRANSFER_P_COEFFICIENT = 2 * Math.PI / TRANSFER_GEAR_RATIO;

        public static final double TRANSFER_MASS = Units.lbsToKilograms(4);
        public static final double TRANSFER_RADIUS = Units.inchesToMeters(3);
        public static final double TRANSFER_MOI = 0.002;

        public static final DCMotor TRANSFER_MOTORS = DCMotor.getKrakenX60(2); // 2 x44's
    }

    public static final class ShooterConstants {
        public enum ShooterState {
            OFF(0),
            FULL(6);

            public final double volts;

            private ShooterState(double volts) {
                this.volts = volts;
            }
        }

        public static final TalonFXConfiguration getShooterConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 40;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static final int SHOOTER_ONE_ID = 21;
        public static final int SHOOTER_TWO_ID = 22;

        public static final double SHOOTER_GEAR_RATIO = 18. / 36; // 0.5:1
        public static final double SHOOTER_P_COEFFICIENT = 2 * Math.PI / SHOOTER_GEAR_RATIO;

        public static final double SHOOTER_MASS = Units.lbsToKilograms(6);
        public static final double SHOOTER_RADIUS = Units.inchesToMeters(2);
        public static final double SHOOTER_MOI = 0.001;

        public static final DCMotor SHOOTER_MOTORS = DCMotor.getKrakenX60(2);

        // 50% of tangential velocity is transferred to game piece
        public static final double EFFICIENCY = 0.5;
        public static final double TANGENTIAL_VELOCITY_AT_12V =
                SHOOTER_P_COEFFICIENT * SHOOTER_RADIUS * EFFICIENCY; // ~32 m/s
        public static final double EJECT_HEIGHT = 0.5; // 0.67
    }

    public static final class HoodConstants {
        public static final TalonFXConfiguration getPivotConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 20;

            config.MotionMagic.MotionMagicCruiseVelocity = 20;
            config.MotionMagic.MotionMagicAcceleration = 75;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            config.Slot0.kG = 0.0;
            config.Slot0.kS = 0.0;
            config.Slot0.kV = 0.0;
            config.Slot0.kA = 0.0;
            config.Slot0.kP = 0.67;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;

            return config;
        }

        public static final int HOOD_ID = 25;
        public static final double HOOD_GEAR_RATIO = (186. / 10.) * (30. / 14.); // ~39.86
        public static final double HOOD_P_COEFFICIENT = 2 * Math.PI / HOOD_GEAR_RATIO;

        public static final double HOOD_STARTING_ANGLE = Units.degreesToRadians(65);
        public static final double HOOD_MIN_ANGLE = Units.degreesToRadians(0);
        public static final double HOOD_MAX_ANGLE = Units.degreesToRadians(90);

        public static final double HOOD_MASS = Units.lbsToKilograms(4);
        public static final double HOOD_LENGTH = Units.inchesToMeters(8);

        public static final DCMotor HOOD_MOTOR = DCMotor.getKrakenX60(1); // x44

        public static final double HOOD_AMPING_ANGLE = Units.degreesToRadians(65);
    }
}
