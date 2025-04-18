package frc.robot.util.Cameras;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class DriveCamera {
    /*This class currently is made to use one camera for a stream but can be set up with a camera
     * server if multiple cameras are needed/ one camera stream need ot change with another.
     * this could be very helful if there is a intake camera and a placment camera that are opposite. 
     * And depending on if the robot has a gamepiece or not you want to see each camera.
     */

    private UsbCamera m_Camera;

    public DriveCamera(
        int CameraID,
        int FrameRate,
        int widthPixels,
        int heightPixels
    ){
    m_Camera = CameraServer.startAutomaticCapture(CameraID);
    m_Camera.setFPS(FrameRate);
    m_Camera.setResolution(widthPixels, heightPixels);
    }
}
