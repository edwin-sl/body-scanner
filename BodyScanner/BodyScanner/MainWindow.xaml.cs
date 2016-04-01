using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Shapes;
using ColladaUsage;
using Microsoft.Kinect;

using System.ComponentModel;

namespace PointCloudWPF
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        const int width = 640, height = 480; //size of the virtual screen
        const int spaceArms = 4;
        const int spaceFace = 2;
        const int spaceBody = 4;
        const int spaceLegs = 4;
        const float degreesH = 31f;
        const float degreesV = 24.3f;
        enum BodyPart { Head, Body, Legs };

        int scanNumber = 0;
        Point3D oldneck, newneck;
        Point3D oldhip, newhip;
        Skeleton skeleton;
        Skeleton oldSkeleton;

        struct BodyBorders
        {
            static public DepthImagePoint neck;
            static public DepthImagePoint shoulderR;
            static public DepthImagePoint shoulderL;
            static public DepthImagePoint hip;
        }


        KinectSensor sensor; //Kinect camera
        List<Point3D> finalData = new List<Point3D>(); //data from the scanned user
        List<Point3D> dataFace = new List<Point3D>();
        List<Point3D> dataBody = new List<Point3D>();
        List<Point3D> dataArms = new List<Point3D>();
        List<Point3D> dataLegs = new List<Point3D>();
        short[] pixelData; //data got from the camera
        WriteableBitmap image; //image built with the camera depth information
        short userID = -1;
        BackgroundWorker scanCreate = new BackgroundWorker();


        public MainWindow()
        {
            InitializeComponent();
            initKinect();
            initBackgroundWorker();
        }

        void initKinect()
        {
            //Init Kinect
            sensor = KinectSensor.KinectSensors[0];
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            sensor.DepthFrameReady += DepthFrameReady;
            sensor.SkeletonStream.Enable();
            //sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
            sensor.SkeletonFrameReady += SkeletonFrameReady;
            sensor.ColorStream.Enable();
            sensor.Start();
            sensor.ElevationAngle = 0;
            pixelData = new short[sensor.DepthStream.FramePixelDataLength];
            image = new WriteableBitmap(width, height, 96, 96, PixelFormats.Bgr32, null);
        }

        private void initBackgroundWorker()
        {
            scanCreate.DoWork += scanCreate_DoWork;
            scanCreate.RunWorkerCompleted += scanCreate_RunWorkerCompleted;
            scanCreate.ProgressChanged += scanCreate_ProgressChanged;

            scanCreate.WorkerReportsProgress = true;
            scanCreate.WorkerSupportsCancellation = true;
        }

        void SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons;

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame == null)
                    return;

                GUIscreen.Children.Clear();
                skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                skeletonFrame.CopySkeletonDataTo(skeletons);
            }

            //Check there is a user in screen
            if (skeletons.Length == 0)
                return;
            if (userID == -1 || skeletons[userID].TrackingState != SkeletonTrackingState.Tracked)
                userID = findPlayer(skeletons);
            if (userID == -1)
                return;

            skeleton = skeletons[userID];

            BodyBorders.neck = sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.ShoulderCenter].Position, DepthImageFormat.Resolution640x480Fps30);
            BodyBorders.shoulderL = sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.ShoulderLeft].Position, DepthImageFormat.Resolution640x480Fps30);
            BodyBorders.shoulderR = sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.ShoulderRight].Position, DepthImageFormat.Resolution640x480Fps30);
            BodyBorders.hip = sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.HipCenter].Position, DepthImageFormat.Resolution640x480Fps30);


            Point head = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.Head].Position, DepthImageFormat.Resolution640x480Fps30));
            Point hip = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.HipCenter].Position, DepthImageFormat.Resolution640x480Fps30));

            Point shoulderL = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.ShoulderLeft].Position, DepthImageFormat.Resolution640x480Fps30));
            Point shoulderR = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.ShoulderRight].Position, DepthImageFormat.Resolution640x480Fps30));
            Point shoulderC = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.ShoulderCenter].Position, DepthImageFormat.Resolution640x480Fps30));

            Point elbowL = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.ElbowLeft].Position, DepthImageFormat.Resolution640x480Fps30));
            Point elbowR = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.ElbowRight].Position, DepthImageFormat.Resolution640x480Fps30));

            Point wristL = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.WristLeft].Position, DepthImageFormat.Resolution640x480Fps30));
            Point wristR = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.WristRight].Position, DepthImageFormat.Resolution640x480Fps30));

            Point handL = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.HandLeft].Position, DepthImageFormat.Resolution640x480Fps30));
            Point handR = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.HandRight].Position, DepthImageFormat.Resolution640x480Fps30));

            Point hipL = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.HipLeft].Position, DepthImageFormat.Resolution640x480Fps30));
            Point hipR = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.HipRight].Position, DepthImageFormat.Resolution640x480Fps30));

            Point kneeL = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.KneeLeft].Position, DepthImageFormat.Resolution640x480Fps30));
            Point kneeR = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.KneeRight].Position, DepthImageFormat.Resolution640x480Fps30));

            Point ankleL = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.AnkleLeft].Position, DepthImageFormat.Resolution640x480Fps30));
            Point ankleR = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.AnkleRight].Position, DepthImageFormat.Resolution640x480Fps30));

            Point footL = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.FootLeft].Position, DepthImageFormat.Resolution640x480Fps30));
            Point footR = ScreenPosition(sensor.MapSkeletonPointToDepth(skeleton.Joints[JointType.FootRight].Position, DepthImageFormat.Resolution640x480Fps30));

            //Console.WriteLine(skeleton.Joints[JointType.Head].Position.X + ", " + skeleton.Joints[JointType.Head].Position.Y + ", " + skeleton.Joints[JointType.Head].Position.Z);
            /*
            drawLine(head, shoulderC);
            drawLine(shoulderL, shoulderC);
            drawLine(shoulderR, shoulderC);
            drawLine(shoulderL, elbowL);
            drawLine(elbowL, wristL);
            drawLine(wristL, handL);
            drawLine(shoulderR, elbowR);
            drawLine(elbowR, wristR);
            drawLine(wristR, handR);
            drawLine(hip, shoulderC);
            drawLine(hipL, hip);
            drawLine(hipR, hip);
            drawLine(hipL, kneeL);
            drawLine(kneeL, ankleL);
            drawLine(ankleL, footL);
            drawLine(hipR, kneeR);
            drawLine(kneeR, ankleR);
            drawLine(ankleR, footR);
            */
            string position = "";
            position += skeleton.Joints[JointType.ShoulderCenter].Position.X + ",\n" + skeleton.Joints[JointType.ShoulderCenter].Position.Y + ",\n" + skeleton.Joints[JointType.ShoulderCenter].Position.Z;

        }

        private short findPlayer(Skeleton[] users)
        {
            for (short i = 0; i < users.Length; i++)
            {
                if (users[i].TrackingState == SkeletonTrackingState.Tracked)
                    return i;
            }
            return -1;
        }

        private void drawLine(Point p1, Point p2)
        {
            Line myLine = new Line();

            myLine.Stroke = System.Windows.Media.Brushes.ForestGreen;
            myLine.Fill = System.Windows.Media.Brushes.ForestGreen;
            myLine.X1 = p1.X;
            myLine.X2 = p2.X;
            myLine.Y1 = p1.Y;
            myLine.Y2 = p2.Y;
            //myLine.HorizontalAlignment = HorizontalAlignment.Left;
            //myLine.VerticalAlignment = VerticalAlignment.Center;
            myLine.StrokeThickness = 5;

            GUIscreen.Children.Add(myLine);
        }

        private Point ScreenPosition(DepthImagePoint p)
        {
            double x, y;

            x = p.X * GUIscreen.ActualWidth / width;
            y = p.Y * GUIscreen.ActualHeight / height;

            return new Point(x, y);
        }

        void DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame imageFrame = e.OpenDepthImageFrame())
            {
                if (imageFrame == null)
                    return;

                imageFrame.CopyPixelDataTo(pixelData);

                createCloudPoint(pixelData);

            }
        }

        //Use data fromKinect to position the deep of each tiangle
        unsafe private void createCloudPoint(short[] depthData)
        {
            ushort depthValue = 0;
            int player = 0;
            ushort precision = 1;
            byte margin = 10;
            byte[] imageData = new byte[width * height * 4];
            int posX, posY;

            fixed (byte* imagePtr = imageData)
            {
                fixed (short* depthPtr = depthData)
                {
                    for (int pos = 0; pos < width * height; pos++)
                    {
                        depthValue = (ushort)(*(depthPtr + pos) >> 3); //depth
                        player = *(depthPtr + pos) & DepthImageFrame.PlayerIndexBitmask; //player id

                        if (player == userID + 1 && player != 0)
                        {
                            posX = pos % width;
                            posY = pos / width;
                            if (depthValue == sensor.DepthStream.TooNearDepth || depthValue == sensor.DepthStream.TooNearDepth ||
                                pos / width <= margin || pos / width >= height - margin || pos % width <= margin || pos % width >= width - margin) // Error Color
                            {
                                //Blue
                                //*(imagePtr + pos * 4) = (byte)255;
                                //Green
                                //*(imagePtr + pos * 4 + 1) = (byte)255;
                                //Red
                                *(imagePtr + pos * 4 + 2) = (byte)255;
                                //Alpha
                            }
                            else if (posY <= BodyBorders.neck.Y && posX > BodyBorders.shoulderL.X && posX < BodyBorders.shoulderR.X) // Scan head
                            {
                                if (depthValue > ((ushort)(*(depthPtr + pos - 1) >> 3) - precision) &&
                                    depthValue > ((ushort)(*(depthPtr + pos + width) >> 3) - precision)) // Siluette Color
                                {
                                    //Blue
                                    *(imagePtr + pos * 4) = (byte)255;
                                    //Green
                                    *(imagePtr + pos * 4 + 1) = (byte)200;
                                    //Red
                                    *(imagePtr + pos * 4 + 2) = (byte)150;
                                    //Alpha
                                }
                                else // Definitions Color
                                {
                                    //Blue
                                    *(imagePtr + pos * 4) = (byte)110;
                                    //Green
                                    *(imagePtr + pos * 4 + 1) = (byte)40;
                                    //Red
                                    *(imagePtr + pos * 4 + 2) = (byte)20;
                                    //Alpha
                                }
                            }
                            else if ((posX <= BodyBorders.shoulderL.X || posX >= BodyBorders.shoulderR.X) && posY < BodyBorders.hip.Y) //Scan arms
                            {
                                if (depthValue > ((ushort)(*(depthPtr + pos - 1) >> 3) - precision) &&
                                    depthValue > ((ushort)(*(depthPtr + pos + width) >> 3) - precision)) // Siluette Color
                                {
                                    //Blue
                                    *(imagePtr + pos * 4) = (byte)150;
                                    //Green
                                    *(imagePtr + pos * 4 + 1) = (byte)200;
                                    //Red
                                    *(imagePtr + pos * 4 + 2) = (byte)255;
                                    //Alpha
                                }
                                else // Definitions Color
                                {
                                    //Blue
                                    *(imagePtr + pos * 4) = (byte)20;
                                    //Green
                                    *(imagePtr + pos * 4 + 1) = (byte)40;
                                    //Red
                                    *(imagePtr + pos * 4 + 2) = (byte)110;
                                    //Alpha
                                }
                            }
                            else if (posY >= BodyBorders.hip.Y) // Scan legs
                            {
                                if (depthValue > ((ushort)(*(depthPtr + pos - 1) >> 3) - precision) &&
                                    depthValue > ((ushort)(*(depthPtr + pos + width) >> 3) - precision)) // Siluette Color
                                {
                                    //Blue
                                    *(imagePtr + pos * 4) = (byte)200;
                                    //Green
                                    *(imagePtr + pos * 4 + 1) = (byte)255;
                                    //Red
                                    *(imagePtr + pos * 4 + 2) = (byte)150;
                                    //Alpha
                                }
                                else // Definitions Color
                                {
                                    //Blue
                                    *(imagePtr + pos * 4) = (byte)40;
                                    //Green
                                    *(imagePtr + pos * 4 + 1) = (byte)110;
                                    //Red
                                    *(imagePtr + pos * 4 + 2) = (byte)20;
                                    //Alpha
                                }
                            }
                            else //Scan body
                            {
                                if (depthValue > ((ushort)(*(depthPtr + pos - 1) >> 3) - precision) &&
                                    depthValue > ((ushort)(*(depthPtr + pos + width) >> 3) - precision)) // Siluette Color
                                {
                                    //Blue
                                    *(imagePtr + pos * 4) = (byte)255;
                                    //Green
                                    *(imagePtr + pos * 4 + 1) = (byte)150;
                                    //Red
                                    *(imagePtr + pos * 4 + 2) = (byte)200;
                                    //Alpha
                                }
                                else // Definitions Color
                                {
                                    //Blue
                                    *(imagePtr + pos * 4) = (byte)110;
                                    //Green
                                    *(imagePtr + pos * 4 + 1) = (byte)20;
                                    //Red
                                    *(imagePtr + pos * 4 + 2) = (byte)40;
                                    //Alpha
                                }
                            }
                        }
                        else
                        {
                            //*(imagePtr + pos * 4) = (int)0;
                            if (depthValue > ((ushort)(*(depthPtr + pos - 1) >> 3) - precision) &&
                                depthValue > ((ushort)(*(depthPtr + pos + width) >> 3) - precision)) // Siluette Color
                            //                                Math.Abs(depthValue - (ushort)(*(depthPtr + pos - 1) >> 3)) < precision &&
                            //                               Math.Abs(depthValue - (ushort)(*(depthPtr + pos + width) >> 3)) < precision)
                            {
                                //Blue
                                *(imagePtr + pos * 4) = (byte)50;
                                //Green
                                *(imagePtr + pos * 4 + 1) = (byte)50;
                                //Red
                                *(imagePtr + pos * 4 + 2) = (byte)50;
                                //Alpha
                            }
                            else // Definitions Color
                            {
                                //Blue
                                *(imagePtr + pos * 4) = (byte)100;
                                //Green
                                *(imagePtr + pos * 4 + 1) = (byte)100;
                                //Red
                                *(imagePtr + pos * 4 + 2) = (byte)100;
                                //Alpha
                            }
                        }

                    }
                }
            }

            image.WritePixels(new Int32Rect(0, 0, width, height), imageData, width * 4, 0);
            ScanImage.Source = image;
        }

        private Point3D convertToMM(int x, int y, int z)
        {
            double newX = 0;
            double newY = 0;
            double HorizontalTanA = Math.Tan(degreesH * Math.PI / 180);
            double VerticalTanA = Math.Abs(Math.Tan(degreesV * Math.PI / 180));

            newX = HorizontalTanA * z / 640;
            newX *= (width / 2 - x);

            newY = VerticalTanA * z / 480;
            newY *= (height / 2 - y);

            return new Point3D(newX, newY, -z);
        }

        unsafe private void fillFinalData(BackgroundWorker worker, int part)
        {
            switch (part)
            {
                case (int)BodyPart.Head:
                    dataFace.Clear();
                    break;
                case (int)BodyPart.Body:
                    dataBody.Clear();

                    dataArms.Clear();
                    break;
                case (int)BodyPart.Legs:
                    dataLegs.Clear();
                    break;
            }
            int posX, posY;

            fixed (short* pixelPtr = pixelData)
            {
                for (int pos = 1; pos < width * height; pos++)
                {

                    if ((*(pixelPtr + pos) & DepthImageFrame.PlayerIndexBitmask) == userID + 1)
                    {
                        posX = pos % width;
                        posY = pos / width;


                        if (posY <= BodyBorders.neck.Y && posX > BodyBorders.shoulderL.X && posX < BodyBorders.shoulderR.X) // Scan head
                        {
                            if (posX % spaceFace == 0 && posY % spaceFace == 0 && part == (int)BodyPart.Head)
                            {
                                dataFace.Add(convertToMM(posX, posY, (ushort)(*(pixelPtr + pos) >> 3)));
                            }
                        }
                        else if ((posX <= BodyBorders.shoulderL.X || posX >= BodyBorders.shoulderR.X) && posY < BodyBorders.hip.Y) //Scan arms
                        {
                            if (posX % spaceArms == 0 && posY % spaceArms == 0 && part == (int)BodyPart.Body)
                            {
                                dataArms.Add(convertToMM(posX, posY, (ushort)(*(pixelPtr + pos) >> 3)));
                            }
                        }
                        else if (posY >= BodyBorders.hip.Y) // Scan legs
                        {
                            if (posX % spaceLegs == 0 && posY % spaceLegs == 0 && part == (int)BodyPart.Legs)
                            {
                                dataLegs.Add(convertToMM(posX, posY, (ushort)(*(pixelPtr + pos) >> 3)));
                            }
                        }
                        else if (part == (int)BodyPart.Body)//Scan body
                        {
                            if (posX % spaceBody == 0 && posY % spaceBody == 0)
                            {
                                dataBody.Add(convertToMM(posX, posY, (ushort)(*(pixelPtr + pos) >> 3)));
                            }
                        }
                    }
                    //worker.ReportProgress(pos * 25 / (width * height));
                }
            }
        }

        private float cutModelBorder(List<Point3D> modelData)
        {
            float desiredDepth = 0;
            float offsetDepth = 90;
            float error = 0;
            int count = 0;

            SortedList<float, int> depths = new SortedList<float, int>();

            for (int i = 1; i < modelData.Count - 1; i++)
            {
                if (modelData[i - 1].Y == modelData[i].Y && modelData[i + 1].Y == modelData[i].Y)
                    continue;

                desiredDepth += (float)modelData[i].Z;
                count++;

                if (depths.ContainsKey((float)(modelData[i].Z / (error + 1))))
                {
                    depths[(float)(modelData[i].Z / (error + 1))]++;

                }
                else
                {
                    depths.Add((float)(modelData[i].Z / (error + 1)), 1);
                }
            }

            /*
            foreach (KeyValuePair<float, int> borderPoint in depths)
            {
                if (borderPoint.Value > count)
                {
                    //desiredDepth = borderPoint.Key;
                    //count = borderPoint.Value;
                }
            }*/
            desiredDepth /= count;

            for (int i = 0; i < modelData.Count - 1; i++)
            {
                //if (modelData[i - 1].Y == modelData[i].Y && modelData[i + 1].Y == modelData[i].Y)
                //    continue;

                if (offsetDepth + modelData[i].Z / (error + 1) < desiredDepth)
                {
                    modelData.RemoveAt(i);
                    i--;
                }
            }

            return desiredDepth;
        }

        private void createPCD(float depthOffset, BackgroundWorker worker)
        {
            String header =
                "VERSION .7\n" +
                "FIELDS x y z\n" +
                "SIZE 4 4 4\n" +
                "TYPE F F F\n" +
                "COUNT 1 1 1\n" +
                "WIDTH " + finalData.Count + "\n" +
                "HEIGHT 1\n" +
                "POINTS " + finalData.Count + "\n" +
                "DATA ascii\n";
            String coord = "";
            //worker.ReportProgress(30);
            int progress = 0;
            foreach (Point3D p in finalData)
            {
                coord += p.X + " " + p.Y + " " + (p.Z - depthOffset) + " \r\n";
                //worker.ReportProgress(30 + progress * 10 / finalData.Count);
            }

            System.IO.StreamWriter file = new System.IO.StreamWriter("coordenadas.txt");
            file.WriteLine(coord + "\n" + finalData.Count);
            file.Close();
            //worker.ReportProgress(45);
            System.IO.StreamWriter file2 = new System.IO.StreamWriter("scanned_body.pcd");
            file2.WriteLine(header + coord);
            file2.Close();
            //worker.ReportProgress(50);
        }

        private void createModelData(BackgroundWorker worker)
        {
            Process PCL_cpp = Process.Start("PCL_EXE(1).exe");
            //worker.ReportProgress(60);
            PCL_cpp.WaitForExit();
            //worker.ReportProgress(75);
        }

        unsafe private void button1_Click(object sender, RoutedEventArgs e)
        {
            if (scanCreate.IsBusy != true)
            {
                scanProgress.Visibility = Visibility.Visible;
                scanNumber++;
                scanCreate.RunWorkerAsync();

            }
        }

        private void joinParts(List<Point3D> limbData, Point3D limbPoint, Point3D bodyPoint)
        {
            SkeletonPoint tempSP = new SkeletonPoint();
            DepthImagePoint temp;

            tempSP.X = (float)limbPoint.X;
            tempSP.Y = (float)limbPoint.Y;
            tempSP.Z = (float)limbPoint.Z;
            temp = sensor.MapSkeletonPointToDepth(tempSP, sensor.DepthStream.Format);
            limbPoint = convertToMM(temp.X, temp.Y, temp.Depth);

            tempSP.X = (float)bodyPoint.X;
            tempSP.Y = (float)bodyPoint.Y;
            tempSP.Z = (float)bodyPoint.Z;
            temp = sensor.MapSkeletonPointToDepth(tempSP, sensor.DepthStream.Format);
            bodyPoint = convertToMM(temp.X, temp.Y, temp.Depth);

            double dX = bodyPoint.X - limbPoint.X;
            double dY = bodyPoint.Y - limbPoint.Y;
            double dZ = bodyPoint.Z - limbPoint.Z;

            Console.WriteLine(limbPoint.ToString() + " -> " + bodyPoint.ToString());
            Point3D tempPoint;
            for (int i = 0; i < limbData.Count; i++)
            {
                tempPoint = new Point3D(limbData[i].X + dX, limbData[i].Y + dY, limbData[i].Z + dZ);

                limbData[i] = tempPoint;

            }
        }

        private void scanCreate_DoWork(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;
            float desiredDepth = 0;

            //fillFinalData(worker);
            if (scanNumber == 1)
            {
                desiredDepth = cutModelBorder(dataFace);
            }
            else
            {
                desiredDepth = cutModelBorder(dataArms);
                desiredDepth = cutModelBorder(dataBody);

                //joinParts();

                finalData.AddRange(dataFace);
                finalData.AddRange(dataArms);
                finalData.AddRange(dataBody);
                finalData.AddRange(dataLegs);

                createPCD(desiredDepth, worker);
                createModelData(worker);
                ColladaFile.CreateFile(worker);
            }
        }

        private void scanCreate_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            scanProgress.Visibility = Visibility.Hidden;
        }

        private void scanCreate_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            scanProgress.Value = e.ProgressPercentage;
        }

        //Kinect look up
        private void button2_Click(object sender, RoutedEventArgs e)
        {
            sensor.ElevationAngle += 2;
        }
        //Kinect look down
        private void button3_Click(object sender, RoutedEventArgs e)
        {
            sensor.ElevationAngle -= 2;
        }

        private void ScanImage_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            GUIscreen.Width = ScanImage.ActualWidth;
            GUIscreen.Height = ScanImage.ActualHeight;
        }

        private void face_btn_Click(object sender, RoutedEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;
            oldneck = new Point3D(skeleton.Joints[JointType.ShoulderCenter].Position.X, skeleton.Joints[JointType.ShoulderCenter].Position.Y, skeleton.Joints[JointType.ShoulderCenter].Position.Z);

            float desiredDepth = 0;

            fillFinalData(worker, (int)BodyPart.Head);

            //cutModelBorder(dataFace);
        }

        private void body_btn_Click(object sender, RoutedEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;
            float desiredDepth = 0;

            newneck = new Point3D(skeleton.Joints[JointType.ShoulderCenter].Position.X, skeleton.Joints[JointType.ShoulderCenter].Position.Y, skeleton.Joints[JointType.ShoulderCenter].Position.Z);

            newhip = new Point3D(skeleton.Joints[JointType.HipCenter].Position.X, skeleton.Joints[JointType.HipCenter].Position.Y, skeleton.Joints[JointType.HipCenter].Position.Z);
            fillFinalData(worker, (int)BodyPart.Body);
            //desiredDepth = cutModelBorder(dataArms);
            //desiredDepth = cutModelBorder(dataBody);
        }

        private void legs_btn_Click(object sender, RoutedEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;
            oldhip = new Point3D(skeleton.Joints[JointType.HipCenter].Position.X, skeleton.Joints[JointType.HipCenter].Position.Y, skeleton.Joints[JointType.HipCenter].Position.Z);

            float desiredDepth = 0;

            fillFinalData(worker, (int)BodyPart.Legs);
        }

        private void scan_btn_Click(object sender, RoutedEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;
            float desiredDepth = 0;

            cutModelBorder(dataFace);
            cutModelBorder(dataBody);
            cutModelBorder(dataArms);
            cutModelBorder(dataLegs);

            joinParts(dataFace, oldneck, newneck);
            joinParts(dataLegs, oldhip, newhip);
            finalData.Clear();
            finalData.AddRange(dataFace);
            finalData.AddRange(dataArms);
            finalData.AddRange(dataBody);
            finalData.AddRange(dataLegs);


            createPCD(desiredDepth, worker);
            createModelData(worker);
            ColladaFile.CreateFile(worker);
        }

        private void fullScan_btn_Click(object sender, RoutedEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;

            fillFinalData(worker, (int)BodyPart.Head);
            fillFinalData(worker, (int)BodyPart.Body);
            fillFinalData(worker, (int)BodyPart.Legs);
            
            oldneck = new Point3D(skeleton.Joints[JointType.ShoulderCenter].Position.X, skeleton.Joints[JointType.ShoulderCenter].Position.Y, skeleton.Joints[JointType.ShoulderCenter].Position.Z);
            newneck = new Point3D(skeleton.Joints[JointType.ShoulderCenter].Position.X, skeleton.Joints[JointType.ShoulderCenter].Position.Y, skeleton.Joints[JointType.ShoulderCenter].Position.Z);
            newhip = new Point3D(skeleton.Joints[JointType.HipCenter].Position.X, skeleton.Joints[JointType.HipCenter].Position.Y, skeleton.Joints[JointType.HipCenter].Position.Z);
            oldhip = new Point3D(skeleton.Joints[JointType.HipCenter].Position.X, skeleton.Joints[JointType.HipCenter].Position.Y, skeleton.Joints[JointType.HipCenter].Position.Z);
        }
    }
}
