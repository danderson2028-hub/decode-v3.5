package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
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
    public static double d=0.0007;

    private MotorEx transfer = new MotorEx("transfer");

    public Command runTransfer(double power){
        return new SetPower(transfer,power).requires(this);
    }
    public Command dynamicTransfer(){
        return new SetPower(transfer, power).requires(this);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void periodic(){

        ActiveOpMode.telemetry().addData("Transfer velocity", transfer.getVelocity());
        ActiveOpMode.telemetry().addData("Transfer power", power);

    }
}
