package org.firstinspires.ftc.teamcode.tests;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;

@Configurable
@TeleOp(name = "Turret test")

public class TurretTest extends NextFTCOpMode {
    public TurretTest(){
        addComponents(
                new SubsystemComponent(),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    private FeedbackCRServoEx turretOne;
    private CRServoEx turretTwo;

    double totalAngle;
    double previousAngle;




    @Override
    public void onInit()    {
        telemetry.addLine("turret test initialized");
        FeedbackCRServoEx turretOne = new FeedbackCRServoEx(
                0.02,
                () -> ActiveOpMode.hardwareMap().analogInput.get("analog"),
                () -> ActiveOpMode.hardwareMap().crservo.get("turret")
        );
        CRServoEx turretTwo = new CRServoEx("turret");

    }

    public void onStartButtonPressed()  {
        double totalAngle = 0.0;
        double previousAngle = 0.0;
    }

    public void updatePosition() {
        double currentAngle = turretOne.getCurrentPosition();
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
        else if (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;

        totalAngle += deltaAngle;
        previousAngle = currentAngle;
    }

    public void onUpdate()  {
        updatePosition();
        telemetry.addData("servo pos radians", totalAngle);
    }

}
