package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;

import java.util.Set;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
@Configurable
public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer(){}

    public double power = 0.5;


    private MotorEx transfer = new MotorEx("transfer");

    public Command runTransfer(double power){
        return new SetPower(transfer,power).requires(this);
    }
    /*
    public Command dynamicTransfer(){
        return new SetPower(transfer, power).requires(this);
    }
    */
    public Command fireClose(){
        return new SequentialGroup(
                new SetPower(transfer,1),
                new Delay(.65),
                new SetPower(transfer,0.0)
        );
    }
    public Command tapFire(){
        return new SequentialGroup(
                new SetPower(transfer,1),
                new Delay(0.04),
                new SetPower(transfer,0.0)
        );
    }
    public Command fireFar(){
        return new SequentialGroup(
                new SetPower(transfer,1),
                new Delay(0.03),
                new SetPower(transfer,0.0),
                new Delay(0.8),
                new SetPower(transfer,1),
                new Delay(0.05),
                new SetPower(transfer,0.0),
                new Delay(0.8),
                new SetPower(transfer,1),
                new Delay(0.05),
                new SetPower(transfer,0.0)

        );
    }

    @Override
    public void initialize(){

    }

    @Override
    public void periodic(){
        ActiveOpMode.telemetry().addData("Transfer power", power);

    }
}
