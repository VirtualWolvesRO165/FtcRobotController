package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

public class MyLimelight {

    Limelight3A limelight;
    public int caseNumber;
    public void Init(Limelight3A limelight3A){
        limelight = limelight3A;
    }

    public void DetectCase(){
        try {
            // Connect to Limelight JSON API
            URL url = new URL("http://limelight.local:5807/json");
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");

            BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
            String inputLine;
            StringBuilder content = new StringBuilder();
            while ((inputLine = in.readLine()) != null) {
                content.append(inputLine);
            }
            in.close();
            conn.disconnect();

            // Parse JSON
            JSONObject obj = new JSONObject(content.toString());
            int tagId = obj.getJSONObject("Results").getInt("tid");

            // Decide case based on tag ID
            if (tagId == 21) {
                caseNumber = 1; // GPP
            } else if (tagId == 22) {
                caseNumber = 2; // PGP
            } else if (tagId == 23) {
                caseNumber = 3; // PPG
            } else {
                caseNumber = 0; // Not one of ours
            }

//            telemetry.addData("Detected Tag ID", tagId);
//            telemetry.addData("Case", caseNumber);
//            telemetry.update();

        } catch (Exception e) {
//            telemetry.addData("Error", e.toString());
//            telemetry.update();
        }

//        sleep(100); // slow down loop
    }
}
