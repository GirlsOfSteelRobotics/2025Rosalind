package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Auto1", group="Robot")
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {

            chassis.moveBackwards(1500, telemetry);
            //limelight adjust
            shooter.shootEncoderBased(7000, telemetry);
            chassis.turnLeft(500, telemetry);
            shooter.intakeStart();
            chassis.moveBackwards(2000, telemetry);
            shooter.intakeStop();

        }
    }
}
