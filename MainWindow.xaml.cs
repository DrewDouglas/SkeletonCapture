//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Collections;
    using System.Xml;
    using System.Text;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Final joint type for simplifying outputing to file
        /// </summary>
        private const JointType Final_Joint = JointType.WristRight;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Holds all of the body frames to be written to the file after completion
        /// </summary>
        private List<BodyFrame> bodyFrames;

        /// <summary>
        /// Flag to indicate button state 
        /// </summary>
        private Boolean collect;

        /// <summary>
        /// user input filename to write to 
        /// </summary>
        private String fileName;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            this.bodyFrames = new List<BodyFrame>();

            this.collect = false;

            this.fileName = "default";

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (collect)
                        bodyFrames.Add(bodyFrame);

                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void textBox_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {
            fileName = filenameTextBox.Text;
        }

        private void endButton_Click(object sender, RoutedEventArgs e)
        {
            collect = false;
            writeOutput();
            bodyFrames.Clear();
        }

        private void startButton_Click(object sender, RoutedEventArgs e)
        {
            collect = true;
        }

        private void writeOutput()
        {
            //Setup Xml Output File:
            XmlWriter writer = XmlWriter.Create(fileName + "_xml.xml");
            writer.WriteStartDocument();
            writer.WriteStartElement("frames");

            writer.WriteStartElement("total_frames");
            writer.WriteString(bodyFrames.Count.ToString());
            writer.WriteEndElement();

            //Setup CSV output file:
            var csv = new StringBuilder();

            // Write header
            var new_line = string.Format("\tFrameNum, \tRelativeTime, \tAnkleLeft.x, \tAnkleLeft.y, \tAnkleLeft.z, \tAnkleRight.x, \tAnkleRight.y, \tAnkleRight.z, " +
                "\tElbowLeft.x, \tElbowLeft.y, \tElbowLeft.z, \tElbowRight.x, \tElbowRight.y, \tElbowRight.z, \tFootLeft.x, \tFootLeft.y, \tFootLeft.z, " +
                "\tFootRight.x, \tFootRight.y, \tFootRight.z , \tHandLeft.x, \tHandLeft.y, \tHandLeft.z, \tHandRight.x, \tHandRight.y, \tHandRight.z, " +
                "\tHandTipLeft.x, \tHandTipLeft.y, \tHandTipLeft.z, \tHandTipRight.x, \tHandTipRight.y, \tHandTipRight.z, \tHead.x, \tHead.y, \tHead.z, " +
                "\tHipLeft.x, \tHipLeft.y, \tHipLeft.z, HipRight.x, \tHipRight.y, \tHipRight.z, \tKneeLeft.x, \tKneeLeft.y, \tKneeLeft.z, " +
                "\tKneeRight.x, \tKneeRight.y, \tKneeRight.z, \tNeck.x, \tNeck.y, \tNeck.z, \tShoulderLeft.x, \tShoulderLeft.y, \tShoulderLeft.z, " +
                "\tShoulderRight.x, \tShoulderRight.y, \tShoulderRight.z, \tSpineBase.x, SpineBase.y, \tSpineBase.z, \tSpineMid.x, \tSpineMid.y, \tSpineMid.z, " +
                "\tSpineShoulder.x, \tSpineShoulder.y, \tSpineShoulder.z, \tThumbLeft.x, ThumbLeft.y, \tThumbLeft.z, \tThumbRight.x, \tThumbRight.y, \tThumbRight.z, " +
                "\tWristLeft.x, \tWristLeft.y, \tWristLeft.z, \tWristRight.x, \tWristRight.y, \tWristRight.z");
            csv.AppendLine(new_line);

            //Let the bodies hit the frames
            BodyFrame curFrame;
            Body curBody;
            List<Body> curBodies = new List<Body>();
            for (int i = 0; i < bodyFrames.Count; i++)
            {
                curFrame = bodyFrames[i];
                curFrame.GetAndRefreshBodyData(curBodies);
                curBody = curBodies[0];

                //Write the frame number and time
                writer.WriteStartElement("FrameNum");
                writer.WriteString(i.ToString());
                writer.WriteEndElement();

                writer.WriteStartElement("RelativeTime");
                writer.WriteString(curFrame.RelativeTime.ToString());
                writer.WriteEndElement();

                csv.AppendFormat("{0},{1},", i.ToString(), curFrame.RelativeTime.ToString());

                //All joints
                writeJoint(writer, csv, curBody, "AnkleLeft", Microsoft.Kinect.JointType.AnkleLeft);
                writeJoint(writer, csv, curBody, "AnkleRight", Microsoft.Kinect.JointType.AnkleRight);
                writeJoint(writer, csv, curBody, "ElbowLeft", Microsoft.Kinect.JointType.ElbowLeft);
                writeJoint(writer, csv, curBody, "ElbowRight", Microsoft.Kinect.JointType.ElbowRight);
                writeJoint(writer, csv, curBody, "FootLeft", Microsoft.Kinect.JointType.FootLeft);

                writeJoint(writer, csv, curBody, "FootRight", Microsoft.Kinect.JointType.FootRight);
                writeJoint(writer, csv, curBody, "HandLeft", Microsoft.Kinect.JointType.HandLeft);
                writeJoint(writer, csv, curBody, "HandRight", Microsoft.Kinect.JointType.HandRight);
                writeJoint(writer, csv, curBody, "HandTipLeft", Microsoft.Kinect.JointType.HandTipLeft);
                writeJoint(writer, csv, curBody, "HandTipRight", Microsoft.Kinect.JointType.HandTipRight);

                writeJoint(writer, csv, curBody, "Head", Microsoft.Kinect.JointType.Head);
                writeJoint(writer, csv, curBody, "HipLeft", Microsoft.Kinect.JointType.HipLeft);
                writeJoint(writer, csv, curBody, "HipRight", Microsoft.Kinect.JointType.HipRight);
                writeJoint(writer, csv, curBody, "KneeLeft", Microsoft.Kinect.JointType.KneeLeft);
                writeJoint(writer, csv, curBody, "KneeRight", Microsoft.Kinect.JointType.KneeRight);

                writeJoint(writer, csv, curBody, "Neck", Microsoft.Kinect.JointType.Neck);
                writeJoint(writer, csv, curBody, "ShoulderLeft", Microsoft.Kinect.JointType.ShoulderLeft);
                writeJoint(writer, csv, curBody, "ShoulderRight", Microsoft.Kinect.JointType.ShoulderRight);
                writeJoint(writer, csv, curBody, "SpineBase", Microsoft.Kinect.JointType.SpineBase);
                writeJoint(writer, csv, curBody, "SpineMid", Microsoft.Kinect.JointType.SpineMid);

                writeJoint(writer, csv, curBody, "SpineShoulder", Microsoft.Kinect.JointType.SpineShoulder);
                writeJoint(writer, csv, curBody, "ThumbLeft", Microsoft.Kinect.JointType.ThumbLeft);
                writeJoint(writer, csv, curBody, "ThumbRight", Microsoft.Kinect.JointType.ThumbRight);
                writeJoint(writer, csv, curBody, "WristLeft", Microsoft.Kinect.JointType.WristLeft);
                writeJoint(writer, csv, curBody, "WristRight", Microsoft.Kinect.JointType.WristRight);
            }

            writer.WriteEndDocument();
            writer.Close();

            File.WriteAllText(fileName + "_csv.csv", csv.ToString());
        }
        private void writeJoint(XmlWriter writerXML, StringBuilder csv, Body curBody, string name, Microsoft.Kinect.JointType jt)
        {
            writerXML.WriteStartElement(name);

            writerXML.WriteStartElement("x");
            writerXML.WriteString(curBody.Joints[jt].Position.X.ToString());
            writerXML.WriteEndElement();

            writerXML.WriteStartElement("y");
            writerXML.WriteString(curBody.Joints[jt].Position.Y.ToString());
            writerXML.WriteEndElement();

            writerXML.WriteStartElement("z");
            writerXML.WriteString(curBody.Joints[jt].Position.Z.ToString());
            writerXML.WriteEndElement();

            writerXML.WriteEndElement();

            csv.AppendFormat("\t{0},\t{1},\t{2}", curBody.Joints[jt].Position.X.ToString(), curBody.Joints[jt].Position.Y.ToString(), curBody.Joints[jt].Position.Z.ToString());
            csv.Append((jt != Final_Joint) ? ',' : '\n', 1);
        }
    }
}
