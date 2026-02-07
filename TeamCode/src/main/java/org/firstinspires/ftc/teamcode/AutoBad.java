package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto1Bad", group="Robot")
public class AutoBad extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();
if (opModeIsActive()) {
    chassis.moveForwardBad(3000, telemetry);
        }
    }
}
