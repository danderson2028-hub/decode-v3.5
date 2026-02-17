package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPositions;

@TeleOp
@Configurable
public class ServoTest extends NextFTCOpMode {
    public ServoTest(){
        addComponents(
                new SubsystemComponent(Hood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }



    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().circle().whenBecomesTrue(Hood.INSTANCE.down);
        Gamepads.gamepad1().triangle().whenBecomesTrue(Hood.INSTANCE.up);
        //Shooter.INSTANCE.runFlywheelMid.schedule();

    }

    @Override
    public void onUpdate(){

    }
}
