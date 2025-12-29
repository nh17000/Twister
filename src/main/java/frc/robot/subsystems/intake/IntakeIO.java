package frc.robot.subsystems.intake;

import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    static class IntakeIOInputs {
        public MotorData rollerData = new MotorData();
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void runPivotPosition(double setpointRots) {}

    default void runRollerVolts(double volts) {}
}
