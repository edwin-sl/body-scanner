using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows.Media;

namespace BodyScanner
{
    class Scanner
    {
        const int width = 640, height = 480;
        KinectSensor sensor; //Kinect camera
        short[] pixelData; //data got from the camera
        WriteableBitmap image; //image built with the camera depth information
        short userID = -1;

        public Scanner()
        {
            initKinect();
        }

        private void initKinect()
        {
            //Init Kinect
            sensor = KinectSensor.KinectSensors[0];
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            //sensor.DepthFrameReady += DepthFrameReady;
            sensor.SkeletonStream.Enable();
            //sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
            //sensor.SkeletonFrameReady += SkeletonFrameReady;
            sensor.ColorStream.Enable();
            sensor.Start();
            sensor.ElevationAngle = 0;
            pixelData = new short[sensor.DepthStream.FramePixelDataLength];
            image = new WriteableBitmap(width, height, 96, 96, PixelFormats.Bgr32, null);
        }


    }
}
