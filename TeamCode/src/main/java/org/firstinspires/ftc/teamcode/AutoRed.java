package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto Red", group="Linear Opmode")
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();

        nyx.drive(-27, 0.5);
        nyx.turn(120, 0.35);
        nyx.drive(-42, 0.5);
        for (DcMotor m : nyx.AllMotors) {
            m.setPower(-0.05);
        }
        nyx.autoDucks(3, -0.5);
        for (DcMotor m : nyx.AllMotors) {
            m.setPower(0);
        }

    }
}
