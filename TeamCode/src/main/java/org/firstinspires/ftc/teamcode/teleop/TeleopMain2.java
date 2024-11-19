package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Colton_coding_thingy (Cadens Remix)")
public class TeleopMain2 extends LinearOpMode {

    private DcMotor bl_drive;
    private DcMotor fl_drive;
    private DcMotor br_drive;
    private DcMotor fr_drive;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     * @authors
     */
    @Override
    public void runOpMode() {
        bl_drive = hardwareMap.get(DcMotor.class, "bl_drive");
        fl_drive = hardwareMap.get(DcMotor.class, "fl_drive");
        br_drive = hardwareMap.get(DcMotor.class, "br_drive");
        fr_drive = hardwareMap.get(DcMotor.class, "fr_drive");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Put initialization blocks here.
        telemetry.addLine("Press start to run the opmode");

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            telemetry.clear();

            bl_drive.setDirection(DcMotor.Direction.REVERSE);
            fl_drive.setDirection(DcMotor.Direction.REVERSE);
            while (opModeIsActive()) {
                // Put loop blocks here.
                /** Driving method
                 * @author Made by colton726, Modified by: evokerking1 */

                if (gamepad1.left_stick_y != 0) {
                    fl_drive.setPower(-gamepad1.left_stick_y);
                    bl_drive.setPower(-gamepad1.left_stick_y);
                    br_drive.setPower(-gamepad1.left_stick_y);
                    fr_drive.setPower(-gamepad1.left_stick_y);
                } else {
                    fl_drive.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    bl_drive.setPower(-(gamepad1.right_trigger - gamepad1.left_trigger));
                    br_drive.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    fr_drive.setPower(-(gamepad1.right_trigger - gamepad1.left_trigger));
                }

                /** Rotation
                 *  @author Made by colton726, Modified by: evokerking1 */

                if (gamepad1.right_stick_x != 0) {
                    fl_drive.setPower(gamepad1.right_stick_x);
                    bl_drive.setPower(gamepad1.right_stick_x);
                    br_drive.setPower(-gamepad1.right_stick_x);
                    fr_drive.setPower(-gamepad1.right_stick_x);
                } else {
                    fl_drive.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    bl_drive.setPower(-(gamepad1.right_trigger - gamepad1.left_trigger));
                    br_drive.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    fr_drive.setPower(-(gamepad1.right_trigger - gamepad1.left_trigger));
                }

                telemetry.addData("Fr wheel speed", fr_drive.getPower());
                telemetry.addData("Fl wheel speed", -fl_drive.getPower());
                telemetry.addData("Br wheel speed", br_drive.getPower());
                telemetry.addData("Bl wheel speed", -bl_drive.getPower());
                telemetry.update();

            }
        }
    }
}
