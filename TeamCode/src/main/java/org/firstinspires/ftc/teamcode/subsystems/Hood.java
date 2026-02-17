package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.commands.Calculations;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

public class Hood implements Subsystem {
    public static final Hood INSTANCE = new Hood();
    private Hood(){}

    private ServoEx hood = new ServoEx("hood");

    public Command up = new SetPosition(hood,0.16).requires(this);
    public Command down = new SetPosition(hood,0.63).requires(this);
    public Command close = new SetPosition(hood,0.5).requires(this);


    @Override
    public void initialize(){

    }

    @Override
    public void periodic(){

    }
}
