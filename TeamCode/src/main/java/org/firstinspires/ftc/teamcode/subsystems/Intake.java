package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake(){}

    private MotorEx intake = new MotorEx("intake");

    //private ControlSystem controlSystem = ControlSystem.builder()
    //        .velPid(0.0003, 0, 0)
    //        .basicFF(0.00038)
    //        .build();
    public Command stopIntake = new SetPower(intake,0.0).requires(this);
    public Command runIntake = new SetPower(intake,-.9).requires(this);
    public Command runIntakeReverse = new SetPower(intake,0.75).requires(this);
    @Override
    public void initialize(){


    }

    @Override
    public void periodic(){
        //ActiveOpMode.telemetry().addData("Intake Velocity", intake.getVelocity());

        //ActiveOpMode.telemetry().addData("Intake power", controlSystem.calculate(intake.getState()));
        //intake.setPower(controlSystem.calculate(intake.getState()));
    }
}
