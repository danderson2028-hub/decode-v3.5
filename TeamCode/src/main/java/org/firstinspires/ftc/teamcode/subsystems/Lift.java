package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.commands.Calculations;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

public class Lift implements Subsystem {
    public static final Lift INSTANCE = new Lift();
    private Lift(){}

    //private FeedbackCRServoEx liftRight = new FeedbackCRServoEx("liftRight");
    private CRServoEx liftLeft = new CRServoEx("liftLeft");


    private ControlSystem controller = ControlSystem.builder()
            .posPid(0)
            .elevatorFF()
            .build();

    @Override
    public void initialize(){

    }

    @Override
    public void periodic(){

    }
}
