package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    @AutoLogOutput
    @Getter
    @Setter
    private IntakeState state = IntakeState.INTAKING;

    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double rollerSetpointVolts = state.rollerVolts;

        io.runRollerVolts(rollerSetpointVolts);

        Logger.recordOutput("Intake/Roller Setpoint Volts", rollerSetpointVolts);
    }

    public double getRollerVelocity() {
        return inputs.rollerData.velocity() * IntakeConstants.ROLLER_P_COEFFICIENT;
    }
}
