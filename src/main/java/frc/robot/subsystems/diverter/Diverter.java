package frc.robot.subsystems.diverter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DiverterConstants;
import frc.robot.Constants.DiverterConstants.DiverterState;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Diverter extends SubsystemBase {
    @AutoLogOutput
    @Getter
    @Setter
    private DiverterState state = DiverterState.STOWED;

    private DiverterIO io;
    private final DiverterIOInputsAutoLogged inputs = new DiverterIOInputsAutoLogged();

    public Diverter(DiverterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Diverter", inputs);

        double pivotSetpointRots =
                (state.pivotRads - DiverterConstants.PIVOT_STARTING_ANGLE) / DiverterConstants.PIVOT_P_COEFFICIENT;
        double rollerSetpointVolts = state.rollerVolts;

        io.runPivotPosition(pivotSetpointRots);
        io.runRollerVolts(rollerSetpointVolts);

        Logger.recordOutput("Diverter/Pivot Setpoint Rots", pivotSetpointRots);
        Logger.recordOutput("Diverter/Roller Setpoint Volts", rollerSetpointVolts);
    }

    @AutoLogOutput
    public double getPivotAngleRadsToHorizontal() {
        return inputs.pivotData.position() * DiverterConstants.PIVOT_P_COEFFICIENT
                + DiverterConstants.PIVOT_STARTING_ANGLE;
    }

    public double getRollerVelocity() {
        return inputs.rollerData.velocity() * DiverterConstants.ROLLER_P_COEFFICIENT;
    }
}
