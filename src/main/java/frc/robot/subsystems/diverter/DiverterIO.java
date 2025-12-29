package frc.robot.subsystems.diverter;

import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface DiverterIO {
    @AutoLog
    static class DiverterIOInputs {
        public MotorData pivotData = new MotorData();
        public MotorData rollerData = new MotorData();
    }

    default void updateInputs(DiverterIOInputs inputs) {}

    default void runPivotPosition(double setpointRots) {}

    default void runRollerVolts(double volts) {}
}
