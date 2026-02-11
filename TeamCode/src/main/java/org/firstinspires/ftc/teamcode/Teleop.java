package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up){
                chassis.aprilTagDrive(telemetry);
            }else{
                chassis.drive(gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);
            }

            if (gamepad1.start) {
                chassis.resetIMU();
            }
            shooter.Shoot(gamepad1.right_trigger>0.1, gamepad1.left_bumper, gamepad1.a, telemetry);
            chassis.limeLight(telemetry);
            updateTelemetry(telemetry);
        }
    }
}
