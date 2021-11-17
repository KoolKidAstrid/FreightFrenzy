package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto BLue", group="Linear Opmode")
public class AutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();

        nyx.setArm(-280);
        sleep(500);

        nyx.drive(-27, 0.5);
        nyx.turn(-90, 0.35);
        nyx.drive(-20, 0.5);
        nyx.turn(-75, 0.35);
        nyx.drive(-23, 0.5);
        for (DcMotor m : nyx.AllMotors) {
            m.setPower(-0.05);
        }
        nyx.autoDucks(3, 0.5);
        for (DcMotor m : nyx.AllMotors) {
            m.setPower(0);
        }
        nyx.drive(18, 0.5);

        nyx.setArm(0);
        
    }
}
