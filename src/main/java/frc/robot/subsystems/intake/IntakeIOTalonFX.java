package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.util.PearadoxTalonFX;

public abstract class IntakeIOTalonFX implements IntakeIO {
    protected final PearadoxTalonFX rollerMotor;

    protected IntakeIOTalonFX() {
        rollerMotor = new PearadoxTalonFX(IntakeConstants.ROLLER_ID, IntakeConstants.getRollerConfig());
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerData = rollerMotor.getData();
    }

    @Override
    public void runRollerVolts(double volts) {
        rollerMotor.setVoltage(volts);
    }
}
