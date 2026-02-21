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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
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
        public static final double BRANCH_SPACING = Units.inchesToMeters(12.97 / 2.0);
        public static final double REEF_ALIGN_TZ = Units.inchesToMeters(22);

        public static final Translation2d LEFT_BRANCH_OFFSET = new Translation2d(REEF_ALIGN_TZ, -BRANCH_SPACING);
        public static final Translation2d RIGHT_BRANCH_OFFSET = new Translation2d(REEF_ALIGN_TZ, BRANCH_SPACING);
        public static final Translation2d MID_OFFSET = new Translation2d(REEF_ALIGN_TZ, 0.0);
        public static final Translation2d STATION_OFFSET = new Translation2d(Units.inchesToMeters(18), 0.0);

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

        public static final double NET_ALIGN_TZ = Units.inchesToMeters(32);
        public static final double MIN_DIST_TO_WALL = Units.inchesToMeters(28);
        public static final double[] NET_RED_Y = {MIN_DIST_TO_WALL, FieldConstants.FIELD_WIDTH / 2.0 - MIN_DIST_TO_WALL
        };
        public static final double[] NET_BLUE_Y = {
            FieldConstants.FIELD_WIDTH / 2.0 + MIN_DIST_TO_WALL, FieldConstants.FIELD_WIDTH - MIN_DIST_TO_WALL
        };
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(651.2);
        public static final double FIELD_WIDTH = Units.inchesToMeters(317.7);

        public static final Pose2d CENTER =
                new Pose2d(FIELD_LENGTH / 2.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.kZero);

        public static final Pose2d BLUE_HUB = new Pose2d(4.5974, 4.034536, Rotation2d.kZero);
        public static final Pose2d RED_HUB = new Pose2d(11.938, 4.034536, Rotation2d.kZero);

        public static final double HUB_HEIGHT = Units.inchesToMeters(72);

        public static final double LOW_FOOTHILL_HEIGHT = Units.inchesToMeters(46.5);
        public static final double HIGH_FOOTHILL_HEIGHT = Units.inchesToMeters(103.5);
        public static final double UPTOWN_HEIGHT = Units.inchesToMeters(68.574);
        public static final double DOWNTOWN_HEIGHT = Units.inchesToMeters(42 + 6); // shoot above opening

        public static final Pose2d LEFT_UPTOWN_DISTRICT =
                new Pose2d(Units.inchesToMeters(204), Units.inchesToMeters(325.882), Rotation2d.kZero);
        public static final Pose2d LEFT_DOWNTOWN_DISTRICT =
                new Pose2d(Units.inchesToMeters(204), Units.inchesToMeters(-1), Rotation2d.kZero);
        public static final double DISTRICT_SEPARATION = Units.inchesToMeters(48);
        public static final int NUM_DISTRICTS = 6; // 6 uptown and 6 downtown districts

        public static final List<Pose2d> FOOTHILL_DISTRICTS = new ArrayList<>();
        public static final List<Pose2d> UPTOWN_DISTRICTS = new ArrayList<>();
        public static final List<Pose2d> DOWNTOWN_DISTRICTS = new ArrayList<>();
        public static final List<Pose2d> HUBS = new ArrayList<>();

        static {
            // east foothills, red protected zone
            FOOTHILL_DISTRICTS.add(new Pose2d(0.4127, 7.196, Rotation2d.kZero));
            FOOTHILL_DISTRICTS.add(new Pose2d(1.223, 7.882, Rotation2d.kZero));

            // west foothills, blue protected zone
            FOOTHILL_DISTRICTS.add(new Pose2d(FIELD_LENGTH - 0.4127, 7.196, Rotation2d.kZero));
            FOOTHILL_DISTRICTS.add(new Pose2d(FIELD_LENGTH - 1.223, 7.882, Rotation2d.kZero));

            for (int i = 0; i < NUM_DISTRICTS; i++) {
                UPTOWN_DISTRICTS.add(
                        LEFT_UPTOWN_DISTRICT.plus(new Transform2d(i * DISTRICT_SEPARATION, 0, Rotation2d.kZero)));

                DOWNTOWN_DISTRICTS.add(
                        LEFT_DOWNTOWN_DISTRICT.plus(new Transform2d(i * DISTRICT_SEPARATION, 0, Rotation2d.kZero)));
            }

            HUBS.add(BLUE_HUB);
            HUBS.add(RED_HUB);
        }

        public static final int BLUE_BUBBLE_ROWS = 5;
        public static final int RED_BUBBLE_ROWS = 5;
        public static final int BLUE_BUBBLE_COLS = 3;
        public static final int RED_BUBBLE_COLS = 3;
        public static final double BUBBLE_ROW_SEPARATION = Units.inchesToMeters(24);
        public static final double BUBBLE_COL_SEPARATION = Units.inchesToMeters(48);
        public static final Pose2d[] BLUE_BUBBLE_STARTING_POSITIONS = new Pose2d[BLUE_BUBBLE_ROWS * BLUE_BUBBLE_COLS];
        public static final Pose2d[] RED_BUBBLE_STARTING_POSITIONS = new Pose2d[RED_BUBBLE_ROWS * BLUE_BUBBLE_COLS];
        public static final Pose2d FIRST_BLUE_BUBBLE_STARTING_POSE =
                new Pose2d(Units.inchesToMeters(204), Units.inchesToMeters(114), Rotation2d.kZero);
        public static final Pose2d FIRST_RED_BUBBLE_STARTING_POSE =
                new Pose2d(Units.inchesToMeters(444), Units.inchesToMeters(114), Rotation2d.kZero);

        static {
            int index = 0;
            for (int row = 0; row < BLUE_BUBBLE_ROWS; row++) {
                for (int col = 0; col < BLUE_BUBBLE_COLS; col++) {
                    BLUE_BUBBLE_STARTING_POSITIONS[index++] =
                            FIRST_BLUE_BUBBLE_STARTING_POSE.transformBy(new Transform2d(
                                    col * BUBBLE_COL_SEPARATION, row * BUBBLE_ROW_SEPARATION, Rotation2d.kZero));
                }
            }
            index = 0;
            for (int row = 0; row < RED_BUBBLE_ROWS; row++) {
                for (int col = 0; col < RED_BUBBLE_COLS; col++) {
                    RED_BUBBLE_STARTING_POSITIONS[index++] = FIRST_RED_BUBBLE_STARTING_POSE.transformBy(new Transform2d(
                            col * -BUBBLE_COL_SEPARATION, row * BUBBLE_ROW_SEPARATION, Rotation2d.kZero));
                }
            }
        }

        // reefscape constants, old
        public static final int[] BLUE_REEF_TAG_IDS = {18, 19, 20, 21, 22, 17};
        public static final int[] BLUE_CORAL_STATION_TAG_IDS = {12, 13};
        public static final int[] RED_REEF_TAG_IDS = {7, 6, 11, 10, 9, 8};
        public static final int[] RED_CORAL_STATION_TAG_IDS = {1, 2};
        public static final int[] ALL_REEF_TAG_IDS = {18, 19, 20, 21, 22, 17, 7, 6, 11, 10, 9, 8};

        public static final List<Pose2d> CORAL_STATIONS = new ArrayList<>();

        static {
            for (int tag : BLUE_CORAL_STATION_TAG_IDS) {
                CORAL_STATIONS.add(
                        VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d());
            }
            for (int tag : RED_CORAL_STATION_TAG_IDS) {
                CORAL_STATIONS.add(
                        VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d());
            }
        }

        public static final Pose3d[] REEF_TAG_POSES = new Pose3d[RED_REEF_TAG_IDS.length + BLUE_REEF_TAG_IDS.length];

        static {
            int i = 0;
            for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
                REEF_TAG_POSES[i++] =
                        VisionConstants.aprilTagLayout.getTagPose(tag).get();
            }
            for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
                REEF_TAG_POSES[i++] =
                        VisionConstants.aprilTagLayout.getTagPose(tag).get();
            }
        }

        public static final List<Pose2d> REEF_TAGS = new ArrayList<>();

        static {
            for (Pose3d tag : REEF_TAG_POSES) {
                REEF_TAGS.add(tag.toPose2d());
            }
        }

        public static final Transform3d HIGH_ALGAE_TRANSFORM =
                new Transform3d(Units.inchesToMeters(-6), 0, Units.inchesToMeters(39.575), Rotation3d.kZero);
        public static final Transform3d LOW_ALGAE_TRANSFORM =
                new Transform3d(Units.inchesToMeters(-6), 0, Units.inchesToMeters(23.675), Rotation3d.kZero);

        public static final Pose3d[] REEF_ALGAE_POSES = new Pose3d[REEF_TAG_POSES.length];

        static {
            for (int i = 0; i < REEF_ALGAE_POSES.length; i++) {
                REEF_ALGAE_POSES[i] = REEF_TAG_POSES[i].plus(i % 2 == 0 ? HIGH_ALGAE_TRANSFORM : LOW_ALGAE_TRANSFORM);
            }
        }

        public static final double BARGE_X = FIELD_LENGTH / 2.0;
        public static final double BARGE_WIDTH = Units.inchesToMeters(40) / 2.0;
        public static final double BARGE_HEIGHT = Units.inchesToMeters(74 + 8);
        public static final double BARGE_HEIGHT_TOLERANCE = Units.inchesToMeters(12);

        public static final Pose2d BLUE_PROCESSOR =
                VisionConstants.aprilTagLayout.getTagPose(16).get().toPose2d();
        public static final Pose2d RED_PROCESSOR =
                VisionConstants.aprilTagLayout.getTagPose(3).get().toPose2d();

        public static final double TRANSLATIONAL_TOLERANCE = Units.inchesToMeters(16);
        public static final double DROP_COOLDOWN = 2.0;
    }

    public static final class VisualizerConstants {
        public static final Translation3d Z0_ZERO = new Translation3d(-0.134550, -0.143323, 0);
        public static final Translation3d Z1_ZERO = new Translation3d(-0.1235075, -0.041317, 0.519888);
        public static final Translation3d Z2_ZERO = new Translation3d(0.024588, 0, 0);
        public static final Translation3d Z3_ZERO = new Translation3d(0.205374, 0, 0.260350);
        public static final Translation3d Z4_ZERO = new Translation3d(0.302910, 0, 0.646415);

        public static final Translation3d Z1_OFFSET = Z1_ZERO.minus(Z0_ZERO);
        public static final Translation3d Z4_OFFSET = Z4_ZERO.minus(Z3_ZERO);

        public static final double TURRET_STARTING_ANGLE = Math.PI / 2;

        public static final double HOOD_STARTING_ANGLE = Units.degreesToRadians(61.549451);
        public static final double HOOD_MIN_ANGLE = Units.degreesToRadians(24.652849);
        public static final double HOOD_MAX_ANGLE = Units.degreesToRadians(69.652849);

        public static final double INTAKE_STARTING_ANGLE = Math.PI / 2;

        public static final double CLIMBER_MAX_DISPLACEMENT = Units.inchesToMeters(5.875);
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
    }

    public static final class IntakeConstants {
        public enum IntakeState {
            STOWED(Units.degreesToRadians(90), 0),
            DEPLOYED(PIVOT_MIN_ANGLE, 6),
            EJECTING(Units.degreesToRadians(45), -6);

            public final double pivotRads;
            public final double rollerVolts;

            private IntakeState(double pivotRads, double rollerVolts) {
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

        public static final int PIVOT_ID = 50;
        public static final double PIVOT_GEAR_RATIO = (42. / 14.) * (66. / 20.) * (50. / 14.); // ~35.36
        public static final double PIVOT_P_COEFFICIENT = 2 * Math.PI / PIVOT_GEAR_RATIO;

        public static final int ROLLER_ID = 51;
        public static final double ROLLER_GEAR_RATIO = (35. / 14.); // 2.5
        public static final double ROLLER_P_COEFFICIENT = 2 * Math.PI / ROLLER_GEAR_RATIO;

        public static final double PIVOT_STARTING_ANGLE = Units.degreesToRadians(90);
        public static final double PIVOT_MIN_ANGLE = Units.degreesToRadians(0);
        public static final double PIVOT_MAX_ANGLE = Units.degreesToRadians(90);

        public static final double PIVOT_MASS = Units.lbsToKilograms(7);
        public static final double PIVOT_LENGTH = Units.inchesToMeters(18);

        public static final double ROLLER_MASS = Units.lbsToKilograms(2);
        public static final double ROLLER_RADIUS = Units.inchesToMeters(1);
        public static final double ROLLER_MOI = 1.0 / 2.0 * ROLLER_MASS * ROLLER_RADIUS * ROLLER_RADIUS;
        public static final double ROLLER_SHAFT_MOI = ROLLER_MOI * ROLLER_GEAR_RATIO * ROLLER_GEAR_RATIO;

        public static final DCMotor PIVOT_MOTOR = DCMotor.getKrakenX60(1);
        public static final DCMotor ROLLER_MOTOR = DCMotor.getKrakenX60(1); // x44
    }

    public static final class SpindexerConstants {
        public enum SpindexerState {
            SPAIN_WITHOUT_THE_SA(0), // ref, start the pin count! (off)
            SPAIN_WITHOUT_THE_IN(1), // enjoy a relaxing retreat... (low speed)
            ESPANA_SIN_EL_PAN(4), // ts (this spindexer) tiene mucha hambre (medium speed)
            SPAIN_WITHOUT_THE_A(12); // spin! (high speed)

            public final double volts;

            private SpindexerState(double volts) {
                this.volts = volts;
            }
        }

        public static final TalonFXConfiguration getSpindexerConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 20;

            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            return config;
        }

        public static final int SPINDEXER_ID = 40;
        public static final double SPINDEXER_GEAR_RATIO = (84. / 12.) * (40. / 32.) * (36. / 14.); // 22.5
        public static final double SPINDEXER_P_COEFFICIENT = 2 * Math.PI / SPINDEXER_GEAR_RATIO;

        // 1 bubble weighs 1.5lbs
        public static final double SPINDEXER_MASS = Units.lbsToKilograms(10);
        // each bubble "orbits" ~6.4 in from the center
        public static final double BUBBLE_TO_SPINDEXER = Units.inchesToMeters(6.379881);
        public static final double SPINDEXER_MOI =
                1.0 / 2.0 * SPINDEXER_MASS * BUBBLE_TO_SPINDEXER * BUBBLE_TO_SPINDEXER;
        public static final double SPINDEXER_SHAFT_MOI = SPINDEXER_MOI * SPINDEXER_GEAR_RATIO * SPINDEXER_GEAR_RATIO;

        public static final DCMotor SPINDEXER_MOTOR = DCMotor.getKrakenX60(1);
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
            ON(3.24); // 2.67

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
        public static final double SHOOTER_MOI = 0.004;

        public static final DCMotor SHOOTER_MOTORS = DCMotor.getKrakenX60(2);

        // 50% of tangential velocity is transferred to game piece
        public static final double EFFICIENCY = 0.5;
        public static final double DUTY_OUT = (ShooterState.ON.volts / 12.);

        public static final double TANGENTIAL_VELOCITY_AT_12V =
                100 * SHOOTER_P_COEFFICIENT * SHOOTER_RADIUS * EFFICIENCY * DUTY_OUT; // ~32/2 m/s
        public static final double EJECT_HEIGHT = 0.67; // 0.635
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

        public static final double HOOD_STARTING_ANGLE = Units.degreesToRadians(61.549451);
        public static final double HOOD_MIN_ANGLE = Units.degreesToRadians(24.652849); // 30
        public static final double HOOD_MAX_ANGLE = Units.degreesToRadians(69.652849); // 75

        public static final double HOOD_MASS = Units.lbsToKilograms(4);
        public static final double HOOD_LENGTH = Units.inchesToMeters(8);

        public static final DCMotor HOOD_MOTOR = DCMotor.getKrakenX60(1); // x44
    }
}
