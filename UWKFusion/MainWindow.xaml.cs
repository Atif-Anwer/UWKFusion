/*------------------------------------------------------------------------------------------ -----------|
 * UNDERWATER KINECT FUSION                                                                             |
 * V2.0                                                                                                 |
 * AUTHOR: ATIF ANWER                                                                                   |
 * DESCRIPTION:                                                                                         |   
 * The following code is the GUI for underwater Kinect Fusion algorithm.                                |
 *                                                                                                      |    
 * Help Sources:                                                                                        |
 * a. http://pterneas.com/2014/02/20/kinect-for-windows-version-2-color-depth-and-infrared-streams/     |
 * b. SDK Samples Color Basics                                                                          |
 * c. Learn the Kinect API Book                                                                         |
 * d. Microsoft Kinect Fusion SDK Demo WPF                                                              |
 *                                                                                                      |
 * Git Source @ BitBucket                                                                               |
 * -----------------------------------------------------------------------------------------------------|*/


using Kinect.KinectFusionExplorer;
using Microsoft.Kinect;
using Microsoft.Kinect.Fusion;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using UWKFusion.Logging;
using Wpf3DTools;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Drawing;
//using System.Drawing.Imaging;
using System.Drawing.Drawing2D;
using System.Runtime.InteropServices;

// Note: System will not work if Microsoft Kinect SDK 2.0 is not isntalled
// since we are using the Kinect SDK classes

/*------------------------------------------------------------------------------------------ */

namespace UWKFusion
{
    public partial class MainWindow : Window, INotifyPropertyChanged, IDisposable
    {

        #region VARIABLE DEFINITIONS

        // Object declaration of Kinect Class
        private KinectSensor myKinectv2 = null;

        // Frame to read multiple types of data - Kinect SDK
        private MultiSourceFrameReader _reader = null;

        // Current status text to display
        private string statusText = null;

        // used to set status of Stats window 
        private bool showStatsWindow = false;

        // used to set status of Stats window 
        private bool showSettingsWindow = false;
        
        // count frames 
        int noOfFrames = 0;

        // to update fps on main screen text box
        StringBuilder txtBox = new StringBuilder();

        // Height and width of screen
        public double _screenHeight;
        public double _screenWidth;
        public double _screenCenterX;
        public double _screenCenterY;

        PixelFormat format = PixelFormats.Bgr32;

        DepthFrame depthFrame = null;
        ColorFrame colorFrame = null;
        InfraredFrame infraredFrame = null;

        /// Intermediate storage for frame data converted to color

        public byte[] depthPixels = null;

        /// Map depth range to byte range
        private const int MapDepthToByte = 8000 / 256;

        // --------------------------------------------------------
        // VARIABLES FROM KINECT FUSION

        // Track whether Dispose has been called
        private bool disposed;

        /// The reconstruction volume processor type. This parameter sets whether AMP or CPU processing is used
        private const ReconstructionProcessor ProcessorType = ReconstructionProcessor.Amp;

        /// The zero-based device index to choose for reconstruction processing if the ReconstructionProcessor AMP options are selected.
        /// Here we automatically choose a device to use for processing by passing -1, 
        private const int DeviceToUse = -1;

        /// If set true, will automatically reset the reconstruction when MaxTrackingErrors have occurred
        private const bool AutoResetReconstructionWhenLost = false;

        /// Max tracking error count, will reset the reconstruction if tracking errors reach the number
        private const int MaxTrackingErrors = 100;

        /// Time threshold to reset the reconstruction if tracking can't be restored within it. This value is valid if GPU is used
        private const int ResetOnTimeStampSkippedMillisecondsGPU = 2000;

        /// Time threshold to reset the reconstruction if tracking can't be restored within it. This value is valid if CPU is used
        private const int ResetOnTimeStampSkippedMillisecondsCPU = 6000;

        /// Width of raw depth stream
        private const int RawDepthWidth = 512;

        /// Height of raw depth stream
        const int RawDepthHeight = 424;

        /// Width of raw color stream
        private const int RawColorWidth = 1920;

        /// Height of raw color stream
        private const int RawColorHeight = 1080;

        /// Downsample factor of the color frame used by the depth visibility test.
        private const int ColorDownsampleFactor = 4;

        /// The height of raw depth stream if keep the w/h ratio as 4:3
        private const int RawDepthHeightWithSpecialRatio = 384;

        /// Event interval for FPS timer
        private const int FpsInterval = 5;

        /// Event interval for status bar timer
        private const int StatusBarInterval = 1;

        /// Force a point cloud calculation and render at least every 100 milliseconds.
        private const int RenderIntervalMilliseconds = 100;

        /// The frame interval where we integrate color. Capturing color has an associated processing cost, so we do not have to capture every frame here.
        private const int ColorIntegrationInterval = 1;

        /// Frame interval we calculate the deltaFromReferenceFrame 
        private const int DeltaFrameCalculationInterval = 2;

        /// Volume Cube and WPF3D Origin coordinate cross axis 3D graphics line thickness in screen pixels
        private const int LineThickness = 2;

        /// WPF3D Origin coordinate cross 3D graphics axis size in m
        private const float OriginCoordinateCrossAxisSize = 0.1f;

        /// Frame interval we update the camera pose finder database.
        private const int CameraPoseFinderProcessFrameCalculationInterval = 5;

        /// How many frames after starting tracking will will wait before starting to store image frames to the pose finder database. 45 successful frames (1.5s).
        private const int MinSuccessfulTrackingFramesForCameraPoseFinder = 45;

        /// How many frames after starting tracking will will wait before starting to store image frames to the pose finder database. 200 successful frames (~7s).
        private const int MinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure = 200;

        /// Here we set a high limit on the maximum residual alignment energy where we consider the tracking to have succeeded. Typically this value would be around 0.2f to 0.3f.
        /// (Lower residual alignment energy after tracking is considered better.)
        private const float MaxAlignToReconstructionEnergyForSuccess = 0.27f;

        /// Here we set a low limit on the residual alignment energy, below which we reject a tracking success report and believe it to have failed. Typically this value would be around 0.005f, as
        /// values below this (i.e. close to 0 which is perfect alignment) most likely come from frames where the majority of the image is obscured (i.e. 0 depth) or mismatched (i.e. similar depths
        /// but different scene or camera pose).
        private const float MinAlignToReconstructionEnergyForSuccess = 0.005f;

        /// Here we set a high limit on the maximum residual alignment energy where we consider the tracking with AlignPointClouds to have succeeded. Typically this value would be around 0.005f to 0.006f.
        /// (Lower residual alignment energy after relocalization is considered better.)
        private const float MaxAlignPointCloudsEnergyForSuccess = 0.006f;

        /// Here we set a low limit on the residual alignment energy, below which we reject a tracking success report from AlignPointClouds and believe it to have failed. This can typically be around 0.
        private const float MinAlignPointCloudsEnergyForSuccess = 0.0f;

        /// The maximum number of matched poseCount we consider when finding the camera pose. Although the matches are ranked, so we look at the highest probability match first, a higher 
        /// value has a greater chance of finding a good match overall, but has the trade-off of being slower. Typically we test up to around the 5 best matches, after which is may be better just
        /// to try again with the next input depth frame if no good match is found.
        private const int MaxCameraPoseFinderPoseTests = 5;

        /// CameraPoseFinderDistanceThresholdReject is a threshold used following the minimum distance calculation between the input frame and the camera pose finder database. This calculated value
        /// between 0 and 1.0f must be less than or equal to the threshold in order to run the pose finder, as the input must at least be similar to the pose finder database for a correct pose to be
        /// matched.
        private const float CameraPoseFinderDistanceThresholdReject = 1.0f; // a value of 1.0 means no rejection

        /// CameraPoseFinderDistanceThresholdAccept is a threshold passed to the ProcessFrame function in the camera pose finder interface. The minimum distance between the input frame and
        /// the pose finder database must be greater than or equal to this value for a new pose to be stored in the database, which regulates how close together poseCount are stored in the database.
        private const float CameraPoseFinderDistanceThresholdAccept = 0.1f;

        /// Maximum residual alignment energy where tracking is still considered successful
        private const int SmoothingKernelWidth = 1; // 0=just copy, 1=3x3, 2=5x5, 3=7x7, here we create a 3x3 kernel

        /// Maximum residual alignment energy where tracking is still considered successful
        private const float SmoothingDistanceThreshold = 0.04f; // 4cm, could use up to around 0.1f;

        /// Maximum translation threshold between successive poses when using AlignPointClouds
        private const float MaxTranslationDeltaAlignPointClouds = 0.3f; // 0.15 - 0.3m per frame typical

        /// Maximum rotation threshold between successive poses when using AlignPointClouds
        private const float MaxRotationDeltaAlignPointClouds = 20.0f; // 10-20 degrees per frame typical

        /// The factor to downsample the depth image by for AlignPointClouds
        private const int DownsampleFactor = 2;

        /// Threshold used in the visibility depth test to check if this depth value occlude an object or not.
        private const ushort DepthVisibilityTestThreshold = 50; // 50mm

        /// Volume Cube 3D graphics line color
        private static System.Windows.Media.Color volumeCubeLineColor = System.Windows.Media.Color.FromArgb(200, 0, 200, 0);   // Green, partly transparent

        /// If set true, will automatically reset the reconstruction when the timestamp changes by
        /// ResetOnTimeStampSkippedMillisecondsGPU or ResetOnTimeStampSkippedMillisecondsCPU for the 
        /// different processor types respectively. This is useful for automatically resetting when
        /// scrubbing through a .XEF file or on loop of a .XEF file during playback. Note that setting
        /// this true may cause constant resets on slow machines that cannot process frames in less
        /// time that the reset threshold. If this occurs, set to false or increase the timeout.
        /// We now try to find the camera pose, however, setting this false will no longer auto reset on .XEF file playback
        private bool autoResetReconstructionOnTimeSkip = false;

        /// Saving mesh flag
        private bool savingMesh;

        /// To display shaded surface normals frame instead of shaded surface frame
        private bool displayNormals;

        /// Capture, integrate and display color when true
        private bool captureColor;

        /// Pause or resume image integration
        public bool pauseIntegration;

        /// Whether render from the live Kinect camera pose or virtual camera pose
        public bool kinectView = true;

        /// Whether render the volume 3D graphics overlay
        public	bool volumeGraphics = false;

        /// Image Width of depth frame
        private int depthWidth = 0;

        /// Image height of depth frame
        private int depthHeight = 0;

        /// Count of pixels in the depth frame
        private int depthPixelCount = 0;

        /// Image width of color frame
        private int colorWidth = 0;

        /// Image height of color frame
        private int colorHeight = 0;

        /// Count of pixels in the color frame
        private int colorPixelCount = 0;

        /// Image width of IR frame
        private int infraredWidth = 0;
                
        /// Image height of IR frame
        private int infraredHeight = 0;

        /// Count of pixels in the IR frame
        private int infraredPixelCount = 0;

        /// The width of the downsampled images for AlignPointClouds
        private int downsampledWidth;

        /// The height of the downsampled images for AlignPointClouds
        private int downsampledHeight;

        /// Store the width of the visibility test map.
        private int depthVisibilityTestMapWidth = 0;

        /// Store the height of the visibility test map.
        private int depthVisibilityTestMapHeight = 0;

        /// The counter for image process failures
        private int trackingErrorCount = 0;

        /// Set true when tracking fails
        private bool trackingFailed;

        /// Set true when tracking fails and stays false until integration resumes.
        private bool trackingHasFailedPreviously;

        /// Set true when the camera pose finder has stored frames in its database and is able to match camera frames.
        private bool cameraPoseFinderAvailable;

        /// The counter for image process successes
        private int successfulFrameCount;

        /// The counter for frames that have been processed
        private int processedFrameCount = 0;

        /// Timestamp of last depth frame
        private TimeSpan lastFrameTimestamp;

        /// Timer to count FPS
        private DispatcherTimer fpsTimer;

        /// Timer stamp of last computation of FPS
        private DateTime lastFPSTimestamp = DateTime.UtcNow;

        /// Timer stamp of last raycast and render
        private DateTime lastRenderTimestamp = DateTime.UtcNow;

        /// Timer used for ensuring status bar message will be displayed at least one second
        private DispatcherTimer statusBarTimer;

        /// Timer stamp of last update of status message
        private DateTime lastStatusTimestamp;

        /// A high priority message queue for status message
        private Queue<string> statusMessageQueue = new Queue<string>();

        /// Intermediate storage for the extended depth data received from the camera in the current frame
        private ushort[] depthImagePixels;

        /// Store the min depth value in color space. Used to prune occlusion.
        private ushort[] depthVisibilityTestMap;

        /// Intermediate storage for the color data received from the camera in 32bit color
        private byte[] colorImagePixels;

        /// Intermediate storage for the color data received from the camera in 32bit color, re-sampled to depth image size
        private int[] resampledColorImagePixels;

        /// Intermediate storage for the color data downsampled from depth image size and used in AlignPointClouds
        private int[] downsampledDeltaFromReferenceColorPixels;

        /// The Kinect Fusion volume, enabling color reconstruction
        private ColorReconstruction volume;

        /// Intermediate storage for the depth float data converted from depth image frame
        private FusionFloatImageFrame depthFloatFrame;

        /// Intermediate storage for the smoothed depth float image frame
        private FusionFloatImageFrame smoothDepthFloatFrame;

        /// Kinect color re-sampled to be the same size as the depth frame
        private FusionColorImageFrame resampledColorFrame;

        /// Kinect color mapped into depth frame
        private FusionColorImageFrame resampledColorFrameDepthAligned;

        /// Per-pixel alignment values
        private FusionFloatImageFrame deltaFromReferenceFrame;

        /// Shaded surface frame from shading point cloud frame
        private FusionColorImageFrame shadedSurfaceFrame;

        /// Shaded surface normals frame from shading point cloud frame
        private FusionColorImageFrame shadedSurfaceNormalsFrame;

        /// Calculated point cloud frame from image integration
        private FusionPointCloudImageFrame raycastPointCloudFrame;

        /// Calculated point cloud frame from input depth
        private FusionPointCloudImageFrame depthPointCloudFrame;

        /// Intermediate storage for the depth float data converted from depth image frame
        private FusionFloatImageFrame downsampledDepthFloatFrame;

        /// Intermediate storage for the depth float data following smoothing
        private FusionFloatImageFrame downsampledSmoothDepthFloatFrame;

        /// Calculated point cloud frame from image integration
        private FusionPointCloudImageFrame downsampledRaycastPointCloudFrame;

        /// Calculated point cloud frame from input depth
        private FusionPointCloudImageFrame downsampledDepthPointCloudFrame;

        /// Kinect color delta from reference frame data from AlignPointClouds
        private FusionColorImageFrame downsampledDeltaFromReferenceFrameColorFrame;

        /// Bitmap contains depth float frame data for rendering
        private WriteableBitmap depthFloatFrameBitmap;

        /// Bitmap contains delta from reference frame data for rendering
        private WriteableBitmap deltaFromReferenceFrameBitmap;

        /// Bitmap contains shaded surface frame data for rendering
        private WriteableBitmap shadedSurfaceFrameBitmap;

        /// Pixel buffer of depth float frame with pixel data in float format
        private float[] depthFloatFrameDepthPixels;

        /// Pixel buffer of delta from reference frame with pixel data in float format
        private float[] deltaFromReferenceFrameFloatPixels;

        /// Pixel buffer of depth float frame with pixel data in 32bit color
        private int[] depthFloatFramePixelsArgb;

        /// Pixel buffer of delta from reference frame in 32bit color
        private int[] deltaFromReferenceFramePixelsArgb;

        /// Pixels buffer of shaded surface frame in 32bit color
        private int[] shadedSurfaceFramePixelsArgb;

        /// Mapping of depth pixels into color image
        private ColorSpacePoint[] colorCoordinates;

        /// Mapped color pixels in depth frame of reference
        private int[] resampledColorImagePixelsAlignedToDepth;

        /// Pixel buffer of depth float frame with pixel data in float format, downsampled for AlignPointClouds
        private float[] downsampledDepthImagePixels;

        /// The coordinate mapper to convert between depth and color frames of reference
        private CoordinateMapper mapper;

        /// Alignment energy from AlignDepthFloatToReconstruction for current frame 
        private float alignmentEnergy;

        /// The worker thread to process the depth and color data
        private Thread workerThread = null;

        /// Event to stop worker thread
        private ManualResetEvent workerThreadStopEvent;

        /// Event to notify that depth data is ready for process
        private ManualResetEvent depthReadyEvent;

        /// Event to notify that color data is ready for process
        private ManualResetEvent colorReadyEvent;

        /// Event to notify that color data is ready for process
        private ManualResetEvent infraredReadyEvent = null;

        /// Lock object for raw pixel access
        private object rawDataLock = new object();

        /// Lock object for volume re-creation and meshing
        private object volumeLock = new object();

        // The volume cube 3D graphical representation
        private ScreenSpaceLines3D volumeCube;

        /// The volume cube 3D graphical representation
        private ScreenSpaceLines3D volumeCubeAxisX;

        /// The volume cube 3D graphical representation
        private ScreenSpaceLines3D volumeCubeAxisY;

        /// The volume cube 3D graphical representation
        private ScreenSpaceLines3D volumeCubeAxisZ;

        /// The axis-aligned coordinate cross X axis
        private ScreenSpaceLines3D axisX;

        /// The axis-aligned coordinate cross Y axis
        private ScreenSpaceLines3D axisY;

        /// The axis-aligned coordinate cross Z axis
        private ScreenSpaceLines3D axisZ;

        /// Indicate whether the 3D view port has added the volume cube
        private bool haveAddedVolumeCube = false;

        /// Indicate whether the 3D view port has added the origin coordinate cross
        private bool haveAddedCoordinateCross = false;

        /// Flag boolean set true to force the reconstruction visualization to be updated after graphics camera movements
        private bool viewChanged = true;

        /// The virtual 3rd person camera view that can be controlled by the mouse
        public GraphicsCamera virtualCamera;

        /// The virtual 3rd person camera view that can be controlled by the mouse - start rotation
        private Quaternion virtualCameraStartRotation = Quaternion.Identity;

        /// The virtual 3rd person camera view that can be controlled by the mouse - start translation
        private Point3D virtualCameraStartTranslation = new Point3D();  // 0,0,0

        /// Flag to signal to worker thread to reset the reconstruction
        public bool resetReconstruction = false;

        /// Flag to signal to worker thread to re-create the reconstruction
        public bool recreateReconstruction = false;

        /// The transformation between the world and camera view coordinate system
        private Matrix4 worldToCameraTransform;

        /// The default transformation between the world and volume coordinate system
        private Matrix4 defaultWorldToVolumeTransform;

        /// Minimum depth distance threshold in meters. Depth pixels below this value will be
        /// returned as invalid (0). Min depth must be positive or 0.
        private float minDepthClip = FusionDepthProcessor.DefaultMinimumDepth;

        /// Maximum depth distance threshold in meters. Depth pixels above this value will be
        /// returned as invalid (0). Max depth must be greater than 0.
        private float maxDepthClip = FusionDepthProcessor.DefaultMaximumDepth;

        /// Image integration weight
        public short integrationWeight = FusionDepthProcessor.DefaultIntegrationWeight;

        /// The reconstruction volume voxel density in voxels per meter (vpm)
        /// 1000mm / 256vpm = ~3.9mm/voxel
        public int voxelsPerMeter = 384;

        /// The reconstruction volume voxel resolution in the X axis
        /// At a setting of 256vpm the volume is 384 / 256 = 1.5m wide
        public int voxelsX = 256;

        /// The reconstruction volume voxel resolution in the Y axis
        /// At a setting of 256vpm the volume is 384 / 256 = 1.5m high
        public int voxelsY = 256;

        /// The reconstruction volume voxel resolution in the Z axis
        /// At a setting of 256vpm the volume is 384 / 256 = 1.5m deep
        public int voxelsZ = 384;

        /// Parameter to translate the reconstruction based on the minimum depth setting. When set to
        /// false, the reconstruction volume +Z axis starts at the camera lens and extends into the scene.
        /// Setting this true in the constructor will move the volume forward along +Z away from the
        /// camera by the minimum depth threshold to enable capture of very small reconstruction volume
        /// by setting a non-identity world-volume transformation in the ResetReconstruction call.
        /// Small volumes should be shifted, as the Kinect hardware has a minimum sensing limit of ~0.35m,
        /// inside which no valid depth is returned, hence it is difficult to initialize and track robustly  
        /// when the majority of a small volume is inside this distance.
        private bool translateResetPoseByMinDepthThreshold = true;

        /// The color mapping of the rendered reconstruction visualization. 
        private Matrix4 worldToBGRTransform = new Matrix4();

        /// The virtual camera pose - updated whenever the user interacts and moves the virtual camera.
        private Matrix4 virtualCameraWorldToCameraMatrix4 = new Matrix4();

        /// Flag set true if at some point color has been captured. 
        /// Used when writing .Ply mesh files to output vertex color.
        private bool colorCaptured = false;

        /// A camera pose finder to store image frames and poseCount to a database then match the input frames
        /// when tracking is lost to help us recover tracking.
        private CameraPoseFinder cameraPoseFinder;

        /// Parameter to enable automatic finding of camera pose when lost. This searches back through
        /// the camera pose history where key-frames and camera poseCount have been stored in the camera
        /// pose finder database to propose the most likely pose matches for the current camera input.
        public bool autoFindCameraPoseWhenLost = true;


        /// Property change event
        public event PropertyChangedEventHandler PropertyChanged;

        /// Gets or sets the timestamp of the current depth frame
        public TimeSpan RelativeTime { get; set; }

        // End vars from Kinect Fusion 

        // Variables from SDK IR Demo

        /// Maximum value (as a float) that can be returned by the InfraredFrame
        private const float InfraredSourceValueMaximum = (float)ushort.MaxValue;

        /// <summary>
        /// The value by which the infrared source data will be scaled
        /// </summary>
        private const float InfraredSourceScale = 0.75f;

        /// <summary>
        /// Smallest value to display when the infrared data is normalized
        /// </summary>
        private const float InfraredOutputValueMinimum = 0.01f;

        /// <summary>
        /// Largest value to display when the infrared data is normalized
        /// </summary>
        private const float InfraredOutputValueMaximum = 1.0f;

        /// Bitmap to display
        /// </summary>
        private WriteableBitmap infraredBitmap = null;

        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;

        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;

        /// Description (width, height, etc) of the infrared frame data
        /// </summary>
        private FrameDescription infraredFrameDescription = null;

        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// Description of the data contained in the color frame
        /// </summary>
        private FrameDescription colorFrameDescription = null;

		/// Depth image is mirrored
		private bool mirrorDepth = true;
		#endregion


		// Generate widnows
		StatsWindow statsWindow = new StatsWindow();

        // Generate widnows
        SettingsWindow settingsWindow = new SettingsWindow();
        // private ushort[] outputImage;
        public ushort[] temp;
		//private BitmapSource writeBmp;


		/*------------------------------------------------------------------------------------------ */

		public MainWindow()
        {
			//DataContext = this;
			this.InitializeComponent();

			//settingsWindow.Owner = this;

			// Setting the data context of the settings window checkboxes to the Main Window
			settingsWindow.chkboxKinectView.DataContext = this;
			settingsWindow.chkboxMirrorDepth.DataContext = this;
			settingsWindow.chkboxVolumeIntegration.DataContext = this;
			settingsWindow.chkboxCameraPoseFinder.DataContext = this;
			settingsWindow.chkboxPauseIntegration.DataContext = this;
			settingsWindow.chkboxCaptureColor.DataContext = this;
			settingsWindow.VolumeIntegrationWeightSlider.DataContext = this;
			settingsWindow.VoxelsXSlider.DataContext = this;
			settingsWindow.VoxelsYSlider.DataContext = this;
			settingsWindow.VoxelsZSlider.DataContext = this;
			settingsWindow.VoxelsPerMeterSlider.DataContext = this;

			
			//statsWindow.Owner = Application.Current.MainWindow;

			// Get screen width, height and center to position the sceens.
			_screenHeight = System.Windows.SystemParameters.WorkArea.Height;
            _screenWidth = System.Windows.SystemParameters.WorkArea.Width;
            _screenCenterX = _screenWidth / 2;
            _screenCenterY = _screenHeight / 2;

            // WIndow Height = "720" Width = "1280"
            this.Top = _screenCenterY - (720 / 2);
            this.Left = _screenCenterX - (1280 / 2);

        }

        /*------------------------------------------------------------------------------------------ */

        ~MainWindow()
        {
            this.Dispose(false);
        }

        /*------------------------------------------------------------------------------------------ */

        public void Dispose()
        {
            this.Dispose(true);

            // This object will be cleaned up by the Dispose method.
            GC.SuppressFinalize(this);
        }

        /*------------------------------------------------------------------------------------------ */

        protected virtual void Dispose(bool disposing)
        {
            if (!this.disposed)
            {
                if (disposing)
                {
                    if (null != this.depthReadyEvent)
                    {
                        this.depthReadyEvent.Dispose();
                    }

                    if (null != this.colorReadyEvent)
                    {
                        this.colorReadyEvent.Dispose();
                    }

                    if (null != this.workerThreadStopEvent)
                    {
                        this.workerThreadStopEvent.Dispose();
                    }

                    this.RemoveVolumeCube3DGraphics();
                    this.DisposeVolumeCube3DGraphics();

                    this.RemoveAxisAlignedCoordinateCross3DGraphics();
                    this.DisposeAxisAlignedCoordinateCross3DGraphics();

                    if (null != this.virtualCamera)
                    {
                        this.virtualCamera.CameraTransformationChanged -= this.OnVirtualCameraTransformationChanged;
                        this.virtualCamera.Detach(this.shadedSurfaceImage);     // Stop getting mouse events from the image
                        this.virtualCamera.Dispose();
                    }

                    this.SafeDisposeFusionResources();

                    if (null != this.volume)
                    {
                        this.volume.Dispose();
                    }
                }

                this.disposed = true;
            }
        }

        /*------------------------------------------------------------------------------------------ */

        private void Window_Closed(object sender, EventArgs e)
        {
            // Stop timer
            if (null != this.fpsTimer)
            {
                this.fpsTimer.Stop();
                this.fpsTimer.Tick -= this.FpsTimerTick;
            }

            if (null != this.statusBarTimer)
            {
                this.statusBarTimer.Stop();
                this.statusBarTimer.Tick -= this.StatusBarTimerTick;
            }

            if (this._reader != null)
            {
                this._reader.Dispose();
                this._reader = null;
            }

            if (null != this.myKinectv2)
            {
                this.myKinectv2.Close();
                this.myKinectv2 = null;
            }

            //// Remove the camera frustum 3D graphics from WPF3D
            //this.virtualCamera.DisposeFrustum3DGraphics();

            // Stop worker thread
            this.StopWorkerThread();

            Application.Current.Shutdown();
        }

        /*------------------------------------------------------------------------------------------ */

        /// Execute startup tasks
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            btnScreenshot.IsEnabled = false;
            btnSavePLY.IsEnabled = false;
            btnStart.IsEnabled = false;

			settingsWindow.Owner = this;
            
        }

        /*------------------------------------------------------------------------------------------ */

        /// Start the work thread to process incoming depth data
        /// </summary>
        private void StartWorkerThread()
        {
            if (null == this.workerThread)
            {
                // Initialize events
                this.depthReadyEvent = new ManualResetEvent(false);
                this.colorReadyEvent = new ManualResetEvent(false);
                this.workerThreadStopEvent = new ManualResetEvent(false);

                // Create worker thread and start
                this.workerThread = new Thread(WorkerThreadProc);
                this.workerThread.Start();
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Worker thread in which depth data is processed
        private void WorkerThreadProc()
        {
            WaitHandle[] events = new WaitHandle[2] { this.workerThreadStopEvent, this.depthReadyEvent };
            while (true)
            {
                int index = WaitHandle.WaitAny(events);

                if (0 == index)
                {
                    // Stop event has been set. Exit thread
                    break;
                }

                // Reset depth ready event
                this.depthReadyEvent.Reset();

                // Pass data to process
                this.Process();
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Stop worker thread
        /// </summary>
        private void StopWorkerThread()
        {
            if (null != this.workerThread)
            {
                // Set stop event to stop thread
                this.workerThreadStopEvent.Set();

                // Wait for exit of thread
                this.workerThread.Join();
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /*###################### The main Kinect Fusion process function#############################*/
        /*-------------------------------------------------------------------------------------------*/
        /*------------------------------------------------------------------------------------------ */
        /*------------------------------------------------------------------------------------------ */
        /*------------------------------------------------------------------------------------------ */

        private void Process()
        {
            //Flag to signal to worker thread to re-create the reconstruction
            if (this.recreateReconstruction)
            {
                // Lock object for volume re-creation and meshing
                lock (this.volumeLock)
                {
                    // set flag and call function
                    this.recreateReconstruction = false;
                    this.RecreateReconstruction();
                }
            }

            // if flag set to reset
            if (this.resetReconstruction)
            {
                this.resetReconstruction = false;
				settingsWindow.resetReconstruction = false;
                ResetReconstruction();
            }

            if (null != this.volume && !this.savingMesh)
            {
                try
                {
                    // Check if camera pose finder is available
                    this.cameraPoseFinderAvailable = this.IsCameraPoseFinderAvailable();

                    // Convert depth to float and render depth frame
                    this.ProcessDepthData();

                    // Track camera pose
                    this.TrackCamera();

                    // Only continue if we do not have tracking errors
                    if (0 == this.trackingErrorCount)
                    {
                        // Integrate depth
                        bool colorAvailable = this.IntegrateData();

                        // Check to see if another depth frame is already available. 
                        // If not we have time to calculate a point cloud and render, 
                        // but if so we make sure we force a render at least every 
                        // RenderIntervalMilliseconds.
                        // AND IF PAUSE INTEGRATION IS NOT FLAGGED 
                        if ((!this.depthReadyEvent.WaitOne(0) || this.IsRenderOverdue) && !this.PauseIntegration)
                        {
                            // Raycast and render
                            this.RenderReconstruction();
                        }

                        // Update camera pose finder, adding key frames to the database
                        if (this.autoFindCameraPoseWhenLost && !this.trackingHasFailedPreviously
                            && this.successfulFrameCount > MinSuccessfulTrackingFramesForCameraPoseFinder
                            && this.processedFrameCount % CameraPoseFinderProcessFrameCalculationInterval == 0
                            && colorAvailable)
                        {
                            this.UpdateCameraPoseFinder();
                        }
                    }
                }
                catch (InvalidOperationException ex)
                {
                    this.ShowStatusMessage(ex.Message);
                }
            }
        }

        /*------------------------------------------------------------------------------------------ */

        /// Re-create the reconstruction object
        /// <returns>Indicate success or failure</returns>
        private bool RecreateReconstruction()
        {
            // Check if sensor has been initialized
            if (null == this.myKinectv2)
            {
                return false;
            }

            // Check Kinect Fusion volume validity - Kinect Fusion Library function
            if (null != this.volume)
            {
                this.volume.Dispose();
                this.volume = null;
            }

            try
            {
                // //     Class is used to setup the volume parameters. Kinect Fusion Library Function
                ReconstructionParameters volParam = new ReconstructionParameters(this.voxelsPerMeter, this.voxelsX, this.voxelsY, this.voxelsZ);

                //  Set the world-view transform to identity, so the world origin is the initial camera location.
                //  Matrix4 is a row-major matrix containing the joint rotation information in the top left 3x3 and zero for translation.
                this.worldToCameraTransform = Matrix4.Identity;

                //     Initialize a Kinect Fusion 3D Reconstruction Volume enabling use with color.
                //     Voxel volume axis sizes must be greater than 0 and a multiple of 32. A Kinect
                //     camera is also required to be connected.
                //     DevoceToUse >> set to -1 to automatically select the default device for a given processor type.
                this.volume = ColorReconstruction.FusionCreateReconstruction(volParam, ProcessorType, DeviceToUse, this.worldToCameraTransform);

                //     Get current internal world-to-volume transform. Note: A right handed coordinate
                //     system is used, with the origin of the volume (i.e. voxel 0,0,0) at the top left
                //     of the front plane of the cube. Similar to bitmap images with top left origin,
                //     +X is to the right, +Y down, and +Z is now forward from origin into the reconstruction
                //     volume. The default transform is a combination of translation in X,Y to locate
                //     the world origin at the center of the front face of the reconstruction volume
                //     cube (with the camera looking onto the volume along +Z), and scaling by the voxelsPerMeter
                //     reconstruction parameter to convert from world coordinate system to volume voxel
                //     indices.
                // Returns:
                //     The current world to volume transform. This is a similarity transformation that
                //     converts world coordinates to volume coordinates.
                this.defaultWorldToVolumeTransform = this.volume.GetCurrentWorldToVolumeTransform();

                /// Parameter to translate the reconstruction based on the minimum depth setting. When set to
                /// false, the reconstruction volume +Z axis starts at the camera lens and extends into the scene.
                /// Setting this true in the constructor will move the volume forward along +Z away from the
                /// camera by the minimum depth threshold to enable capture of very small reconstruction volume
                if (this.translateResetPoseByMinDepthThreshold)
                {
                    this.ResetReconstruction();
                }
                else
                {
                    this.ResetTracking();
                    this.ResetColorImage();
                }

                // Map world X axis to blue channel, Y axis to green channel and Z axis to red channel,
                // normalizing each to the range [0, 1]. We also add a shift of 0.5 to both X,Y channels
                // as the world origin starts located at the center of the front face of the volume,
                // hence we need to map negative x,y world vertex locations to positive color values.
                this.worldToBGRTransform = Matrix4.Identity;
                this.worldToBGRTransform.M11 = this.voxelsPerMeter / this.voxelsX;
                this.worldToBGRTransform.M22 = this.voxelsPerMeter / this.voxelsY;
                this.worldToBGRTransform.M33 = this.voxelsPerMeter / this.voxelsZ;
                this.worldToBGRTransform.M41 = 0.5f;
                this.worldToBGRTransform.M42 = 0.5f;
                this.worldToBGRTransform.M44 = 1.0f;

                // Update the graphics volume cube rendering
                if (this.volumeGraphics)
                {
                    Dispatcher.BeginInvoke(
                        (Action)(() =>
                        {
                            // re-create the volume cube display with the new size
                            this.RemoveVolumeCube3DGraphics();
                            this.DisposeVolumeCube3DGraphics();
                            this.CreateCube3DGraphics(volumeCubeLineColor, LineThickness, new Vector3D(0, 0, 0));
                            this.AddVolumeCube3DGraphics();
                        }));
                }

                // Signal that a render is required
                this.viewChanged = true;

                return true;
            }
            catch (ArgumentException)
            {
                this.volume = null;
                //this.ShowStatusMessage(Properties.Resources.VolumeResolution);
                this.ShowStatusMessage("Volume resolution should be greater than 0 and multiple of 32"); 
            }
            catch (InvalidOperationException ex)
            {
                this.volume = null;
                this.ShowStatusMessage(ex.Message);
            }
            catch (DllNotFoundException)
            {
                this.volume = null;
                //this.ShowStatusMessage(Properties.Resources.MissingPrerequisite);
                this.ShowStatusMessage("A prerequisite component for Kinect Fusion is missing. Please refer to the Toolkit documentation for assistance"); 
            }
            catch (OutOfMemoryException)
            {
                this.volume = null;
                //this.ShowStatusMessage(Properties.Resources.OutOfMemory);
                this.ShowStatusMessage("Out of memory error initializing reconstruction - try a smaller reconstruction volume"); 
            }

            return false;
        }

        /*------------------------------------------------------------------------------------------ */
        // called from RecreateReconstruction();

        public void ResetReconstruction()
        {
            if (null == this.myKinectv2)
            {
                return;
            }

            // Reset tracking error counter
            this.trackingErrorCount = 0;

            // Set the world-view transform to identity, so the world origin is the initial camera location.
            this.worldToCameraTransform = Matrix4.Identity;

            // Reset volume
            if (null != this.volume)
            {
                try
                {
                    // Translate the reconstruction volume location away from the world origin by an amount equal
                    // to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
                    // If set false, the default world origin is set to the center of the front face of the 
                    // volume, which has the effect of locating the volume directly in front of the initial camera
                    // position with the +Z axis into the volume along the initial camera direction of view.
                    if (this.translateResetPoseByMinDepthThreshold)
                    {
                        Matrix4 worldToVolumeTransform = this.defaultWorldToVolumeTransform;

                        // Translate the volume in the Z axis by the minDepthClip distance
                        float minDist = (this.minDepthClip < this.maxDepthClip) ? this.minDepthClip : this.maxDepthClip;
                        worldToVolumeTransform.M43 -= minDist * this.voxelsPerMeter;

                        this.volume.ResetReconstruction(this.worldToCameraTransform, worldToVolumeTransform);
                    }
                    else
                    {
                        //     Clear the volume, and set a new world-to-camera transform (camera view pose)
                        //     or identity. This internally sets the default world-to-volume transform. where
                        //     the Kinect camera is translated in X,Y to the center of the front face of the
                        //     volume cube, looking into the cube, and the world coordinates are scaled to volume
                        //     indices according to the voxels per meter setting.
                        //      Kinect Fusion Library
                        this.volume.ResetReconstruction(this.worldToCameraTransform);
                    }

                    this.ResetTracking();
                    this.ResetColorImage();
                }
                catch (InvalidOperationException)
                {
                    //this.ShowStatusMessage(Properties.Resources.ResetFailed);
                    this.ShowStatusMessage("Kinect Fusion reset reconstruction call failed");
                }
            }
        }

        /*------------------------------------------------------------------------------------------ */

        // Reset tracking variables
        // called from ResetReconstruction();
        private void ResetTracking()
        {
            this.trackingFailed = false;
            this.trackingHasFailedPreviously = false;
            this.trackingErrorCount = 0;
            this.successfulFrameCount = 0;

            // Reset pause and signal that the integration resumed
            this.PauseIntegration = false;

            if (null != this.cameraPoseFinder)
            {
                // camera pose finder to store image frames and poseCount to a database then match the input frames
                this.cameraPoseFinder.ResetCameraPoseFinder();
            }
        }

        /*------------------------------------------------------------------------------------------ */

        /// Reset the mapped color image on reset or re-create of volume
        /// called from ResetReconstruction();
        private void ResetColorImage()
        {
            if (null != this.resampledColorFrameDepthAligned && null != this.resampledColorImagePixelsAlignedToDepth)
            {
                // Clear the mapped color image
                Array.Clear(this.resampledColorImagePixelsAlignedToDepth, 0, this.resampledColorImagePixelsAlignedToDepth.Length);
                this.resampledColorFrameDepthAligned.CopyPixelDataFrom(this.resampledColorImagePixelsAlignedToDepth);
            }

            this.colorCaptured = false;
        }

        /*------------------------------------------------------------------------------------------ */

        /// Show high priority messages in status bar
        private void ShowStatusMessage(string message)
        {
            this.Dispatcher.BeginInvoke(
               (Action)(() =>
               {
                   this.ResetFps();

                   if ((DateTime.Now - this.lastStatusTimestamp).Seconds >= StatusBarInterval)
                   {
                       this.statusBarText.Text = message;
					   debugLog( message );
                       this.lastStatusTimestamp = DateTime.Now;
                   }
                   else
                   {
                       this.statusMessageQueue.Enqueue(message);
                   }
               }));
        }

        /*------------------------------------------------------------------------------------------ */
        /// Reset FPS timer and counter
        /// called from ShowStatusMessage();
        private void ResetFps()
        {
            // Restart fps timer
            if (null != this.fpsTimer)
            {
                this.fpsTimer.Stop();
                this.fpsTimer.Start();
            }

            // Reset frame counter
            this.processedFrameCount = 0;
            this.lastFPSTimestamp = DateTime.UtcNow;
        }

        /*------------------------------------------------------------------------------------------ */
        /// Remove the volume cube and axes from the visual tree
        /// Called from RecreateReconstruction();
        private void RemoveVolumeCube3DGraphics()
        {
            if (null != this.volumeCube)
            {
                this.GraphicsViewport.Children.Remove(this.volumeCube);
            }

            if (null != this.volumeCubeAxisX)
            {
                this.GraphicsViewport.Children.Remove(this.volumeCubeAxisX);
            }

            if (null != this.volumeCubeAxisY)
            {
                this.GraphicsViewport.Children.Remove(this.volumeCubeAxisY);
            }

            if (null != this.volumeCubeAxisZ)
            {
                this.GraphicsViewport.Children.Remove(this.volumeCubeAxisZ);
            }

            this.haveAddedVolumeCube = false;
        }

        /*------------------------------------------------------------------------------------------ */
        /// Dispose the volume cube and axes
        /// called from RecreateReconstruction();
        private void DisposeVolumeCube3DGraphics()
        {
            if (this.haveAddedVolumeCube)
            {
                this.RemoveVolumeCube3DGraphics();
            }

            if (null != this.volumeCube)
            {
                this.volumeCube.Dispose();
                this.volumeCube = null;
            }

            if (null != this.volumeCubeAxisX)
            {
                this.volumeCubeAxisX.Dispose();
                this.volumeCubeAxisX = null;
            }

            if (null != this.volumeCubeAxisY)
            {
                this.volumeCubeAxisY.Dispose();
                this.volumeCubeAxisY = null;
            }

            if (null != this.volumeCubeAxisZ)
            {
                this.volumeCubeAxisZ.Dispose();
                this.volumeCubeAxisZ = null;
            }
        }

        /*------------------------------------------------------------------------------------------ */

        /// Create an axis-aligned volume cube for rendering.
        /// <param name="color">The color of the volume cube.</param>
        /// <param name="thickness">The thickness of the lines in screen pixels.</param>
        /// <param name="translation">World to volume translation vector.</param>
        private void CreateCube3DGraphics(System.Windows.Media.Color color, int thickness, Vector3D translation)
        {
            // Scaler for cube size
            float cubeSizeScaler = 1.0f;

            // Before we created a volume which contains the head
            // Here we create a graphical representation of this volume cube
            float oneOverVpm = 1.0f / this.voxelsPerMeter;

            // This cube is world axis aligned
            float cubeSideX = this.voxelsX * oneOverVpm * cubeSizeScaler;
            float halfSideX = cubeSideX * 0.5f;

            float cubeSideY = this.voxelsY * oneOverVpm * cubeSizeScaler;
            float halfSideY = cubeSideY * 0.5f;

            float cubeSideZ = this.voxelsZ * oneOverVpm * cubeSizeScaler;
            float halfSideZ = cubeSideZ * 0.5f;

            // The translation vector is from the origin to the volume front face
            // And here we describe the translation Z as from the origin to the cube center
            // So we continue to translate half volume size align Z
            translation.Z -= halfSideZ / cubeSizeScaler;

            this.volumeCube = new ScreenSpaceLines3D();
            this.volumeCube.Points = new Point3DCollection();

            // Front face
            // TL front - TR front
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));

            // TR front - BR front
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));

            // BR front - BL front
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));

            // BL front - TL front
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));

            // Rear face
            // TL rear - TR rear
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));

            // TR rear - BR rear
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));

            // BR rear - BL rear
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));

            // BL rear - TL rear
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));

            // Connecting lines
            // TL front - TL rear
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));

            // TR front - TR rear
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));

            // BR front - BR rear
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));

            // BL front - BL rear
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));

            this.volumeCube.Thickness = thickness;
            this.volumeCube.Color = color;

            this.volumeCubeAxisX = new ScreenSpaceLines3D();

            this.volumeCubeAxisX.Points = new Point3DCollection();
            this.volumeCubeAxisX.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCubeAxisX.Points.Add(new Point3D(-halfSideX + 0.1f + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));

            this.volumeCubeAxisX.Thickness = thickness + 2;
            this.volumeCubeAxisX.Color = System.Windows.Media.Color.FromArgb(200, 255, 0, 0);   // Red (X)

            this.volumeCubeAxisY = new ScreenSpaceLines3D();

            this.volumeCubeAxisY.Points = new Point3DCollection();
            this.volumeCubeAxisY.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCubeAxisY.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY - 0.1f + translation.Y, halfSideZ + translation.Z));

            this.volumeCubeAxisY.Thickness = thickness + 2;
            this.volumeCubeAxisY.Color = System.Windows.Media.Color.FromArgb(200, 0, 255, 0);   // Green (Y)

            this.volumeCubeAxisZ = new ScreenSpaceLines3D();

            this.volumeCubeAxisZ.Points = new Point3DCollection();
            this.volumeCubeAxisZ.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCubeAxisZ.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ - 0.1f + translation.Z));

            this.volumeCubeAxisZ.Thickness = thickness + 2;
            this.volumeCubeAxisZ.Color = System.Windows.Media.Color.FromArgb(200, 0, 0, 255);   // Blue (Z)
        }

        /*------------------------------------------------------------------------------------------ */
        /// Add the volume cube and axes to the visual tree
        private void AddVolumeCube3DGraphics()
        {
            if (this.haveAddedVolumeCube)
            {
                return;
            }

            if (null != this.volumeCube)
            {
                this.GraphicsViewport.Children.Add(this.volumeCube);

                this.haveAddedVolumeCube = true;
            }

            if (null != this.volumeCubeAxisX)
            {
                this.GraphicsViewport.Children.Add(this.volumeCubeAxisX);
            }

            if (null != this.volumeCubeAxisY)
            {
                this.GraphicsViewport.Children.Add(this.volumeCubeAxisY);
            }

            if (null != this.volumeCubeAxisZ)
            {
                this.GraphicsViewport.Children.Add(this.volumeCubeAxisZ);
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Is the camera pose finder initialized and running.
        /// <returns>Returns true if available, otherwise false</returns>
        private bool IsCameraPoseFinderAvailable()
        {
            return this.autoFindCameraPoseWhenLost
                && null != this.cameraPoseFinder
                && this.cameraPoseFinder.GetStoredPoseCount() > 0;
        }

        /*------------------------------------------------------------------------------------------ */

        // called by Process();
        private void ProcessDepthData()
        {
            // To enable playback of a .xef file through Kinect Studio and reset of the reconstruction
            // if the .xef loops, we test for when the frame timestamp has skipped a large number. 
            // Note: this will potentially continually reset live reconstructions on slow machines which
            // cannot process a live frame in less time than the reset threshold. Increase the number of
            // milliseconds if this is a problem.
            if (this.autoResetReconstructionOnTimeSkip)
            {
                this.CheckResetTimeStamp(this.RelativeTime);
            }


			/*********************************************/
			// Setting custom clip depths <<
			//maxDepthClip = 1.0f;
			//minDepthClip = 0.20f;
			/*********************************************/

			if ( settingsWindow.ClipNoisyData == true )
			{
				this.maxDepthClip = 1.0f;    // 1000 = 3.28ft
				this.minDepthClip = 0.25f;     // 450 = 1.47ft

				//if ( settingsWindow.somethinChanged == true && settingsWindow.ClipNoisyData == true )
				//{
				//	debugLog( "Clippng data to 1000mm depth" );
				//}

			}
			else
			{
				this.maxDepthClip = 8.0f;
				this.minDepthClip = 0.5f;
				//if ( settingsWindow.somethinChanged == true && settingsWindow.ClipNoisyData == true )
				//{
				//	debugLog( "Using full depth range of 8000mm" );
				//}
			}

			//// Lock the depth operations
			lock (this.rawDataLock)
            {
                this.volume.DepthToDepthFloatFrame(
                    this.depthImagePixels,
                    this.depthFloatFrame,
                    this.minDepthClip,
                    this.maxDepthClip,
                    this.MirrorDepth);
            }

            // Render depth float frame to GUI
            // Frame: xamlFrame_DisplayDepth
            this.Dispatcher.BeginInvoke(
                (Action)
                (() =>
                    this.RenderDepthFloatImage(
                        ref this.depthFloatFrameBitmap,
                        this.xamlFrame_DisplayDepth)));
        }

        /*------------------------------------------------------------------------------------------ */

        /// Check if the gap between 2 frames has reached reset time threshold. If yes, reset the reconstruction
        /// <param name="frameTimestamp">The frame's timestamp.</param>
        ///  called by ProcessDepthData();
        ///  
        private void CheckResetTimeStamp(TimeSpan frameTimestamp)
        {
            if (!this.lastFrameTimestamp.Equals(TimeSpan.Zero))
            {
                long timeThreshold = (ReconstructionProcessor.Amp == ProcessorType) ? ResetOnTimeStampSkippedMillisecondsGPU : ResetOnTimeStampSkippedMillisecondsCPU;

                // Calculate skipped milliseconds between 2 frames
                long skippedMilliseconds = (long)frameTimestamp.Subtract(this.lastFrameTimestamp).Duration().TotalMilliseconds;

                if (skippedMilliseconds >= timeThreshold)
                {
                    //this.ShowStatusMessage(Properties.Resources.ResetVolume);
                    this.ShowStatusMessage("Reconstruction has been reset"); 
                    this.resetReconstruction = true;
                }
            }

			// Set timestamp of last frame
			this.lastFrameTimestamp = frameTimestamp;
		}

        /*------------------------------------------------------------------------------------------ */
        /// Render Fusion depth float frame to UI
        /// </summary>
        /// <param name="bitmap">Bitmap contains depth float frame data for rendering</param>
        /// <param name="image">UI image component to render depth float frame to</param>
        /// called by ProcessDepthData();
        private void RenderDepthFloatImage(ref WriteableBitmap bitmap, System.Windows.Controls.Image image)
        {
            if (null == this.depthFloatFrame)
            {
                return;
            }

            if (null == bitmap || this.depthFloatFrame.Width != bitmap.Width || this.depthFloatFrame.Height != bitmap.Height)
            {
                // Create bitmap of correct format
                bitmap = new WriteableBitmap(this.depthFloatFrame.Width, this.depthFloatFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set bitmap as source to UI image object
                image.Source = bitmap;
            }

            this.depthFloatFrame.CopyPixelDataTo(this.depthFloatFrameDepthPixels);

            // Calculate color of pixels based on depth of each pixel
            float range = 1.0f;
            float oneOverRange = (1.0f / range) * 256.0f;
            float minRange = 0.0f;

            Parallel.For(
            0,
            this.depthFloatFrame.Height,
            y =>
            {
                int index = y * this.depthFloatFrame.Width;
                for (int x = 0; x < this.depthFloatFrame.Width; ++x, ++index)
                {
                    float depth = this.depthFloatFrameDepthPixels[index];
                    int intensity = (depth >= minRange) ? ((byte)((depth - minRange) * oneOverRange)) : 0;

                    this.depthFloatFramePixelsArgb[index] = (255 << 24) | (intensity << 16) | (intensity << 8) | intensity; // set blue, green, red
                }
            });

            // Copy colored pixels to bitmap
            bitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthFloatFrame.Width, this.depthFloatFrame.Height),
                        this.depthFloatFramePixelsArgb,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }

        /*------------------------------------------------------------------------------------------ */

        /// Track the camera pose
        private void TrackCamera()
        {
            bool calculateDeltaFrame = this.processedFrameCount % DeltaFrameCalculationInterval == 0;
            bool trackingSucceeded = false;

            // Get updated camera transform from image alignment
            Matrix4 calculatedCameraPos = this.worldToCameraTransform;

            // Here we can either call TrackCameraAlignDepthFloatToReconstruction or TrackCameraAlignPointClouds
            // The TrackCameraAlignPointClouds function typically has higher performance with the camera pose finder 
            // due to its wider basin of convergence, enabling it to more robustly regain tracking from nearby poses
            // suggested by the camera pose finder after tracking is lost.
            if (this.autoFindCameraPoseWhenLost)
            {
                // Track using AlignPointClouds
                trackingSucceeded = this.TrackCameraAlignPointClouds(ref calculateDeltaFrame, ref calculatedCameraPos);
            }
            else
            {
                // Track using AlignDepthFloatToReconstruction
                trackingSucceeded = this.TrackCameraAlignDepthFloatToReconstruction(calculateDeltaFrame, ref calculatedCameraPos);
            }

            if (!trackingSucceeded && 0 != this.successfulFrameCount)
            {
                this.SetTrackingFailed();

                if (!this.cameraPoseFinderAvailable)
                {
                    // Show tracking error on status bar
                    //this.ShowStatusMessageLowPriority(Properties.Resources.CameraTrackingFailed);
                    this.ShowStatusMessageLowPriority(": Kinect Fusion camera tracking failed. Align the camera to the last tracked position."); 
                }
                else
                {
                    // Here we try to find the correct camera pose, to re-localize camera tracking.
                    // We can call either the version using AlignDepthFloatToReconstruction or the
                    // version using AlignPointClouds, which typically has a higher success rate.
                    // trackingSucceeded = this.FindCameraPoseAlignDepthFloatToReconstruction();
                    trackingSucceeded = this.FindCameraPoseAlignPointClouds();

                    if (!trackingSucceeded)
                    {
                        // Show tracking error on status bar
                        //this.ShowStatusMessageLowPriority(Properties.Resources.CameraTrackingFailed);
                        this.ShowStatusMessageLowPriority(": Kinect Fusion camera tracking failed. Align the camera to the last tracked position.");
                    }
                }
            }
            else
            {
                if (this.trackingHasFailedPreviously)
                {
                    this.ShowStatusMessageLowPriority(": Kinect Fusion camera tracking RECOVERED! Residual energy=" + string.Format(CultureInfo.InvariantCulture, "{0:0.00000}", this.alignmentEnergy));
                }

                this.UpdateAlignDeltas();

                this.SetTrackingSucceeded();

                this.worldToCameraTransform = calculatedCameraPos;
            }

            if (AutoResetReconstructionWhenLost && !trackingSucceeded
                && this.trackingErrorCount >= MaxTrackingErrors)
            {
                // Bad tracking
                //this.ShowStatusMessage(Properties.Resources.ResetVolumeAuto);
                this.ShowStatusMessage(": Kinect Fusion camera tracking failed, automatically reset volume");

                // Automatically Clear Volume and reset tracking if tracking fails
                this.ResetReconstruction();
            }

            if (trackingSucceeded)
            {
                if (this.kinectView)
                {
                    Dispatcher.BeginInvoke((Action)(() => this.UpdateVirtualCameraTransform()));
                }
                else
                {
                    // Just update the frustum
                    Dispatcher.BeginInvoke((Action)(() => this.virtualCamera.UpdateFrustumTransformMatrix4(this.worldToCameraTransform)));
                }

                // Increase processed frame counter
                this.processedFrameCount++;
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Track camera pose using AlignPointClouds
        /// <param name="calculateDeltaFrame">A flag to indicate it is time to calculate the delta frame.</param>
        /// <param name="calculatedCameraPose">The calculated camera pose.</param>
        /// <returns>Returns true if tracking succeeded, false otherwise.</returns>
        private bool TrackCameraAlignPointClouds(ref bool calculateDeltaFrame, ref Matrix4 calculatedCameraPose)
        {
            bool trackingSucceeded = false;

            this.DownsampleDepthFrameNearestNeighbor(this.downsampledDepthFloatFrame, DownsampleFactor);

            // Smooth the depth frame
            this.volume.SmoothDepthFloatFrame(this.downsampledDepthFloatFrame, this.downsampledSmoothDepthFloatFrame, SmoothingKernelWidth, SmoothingDistanceThreshold);

            // Calculate point cloud from the smoothed frame
            FusionDepthProcessor.DepthFloatFrameToPointCloud(this.downsampledSmoothDepthFloatFrame, this.downsampledDepthPointCloudFrame);

            // Get the saved pose view by raycasting the volume from the current camera pose
            this.volume.CalculatePointCloud(this.downsampledRaycastPointCloudFrame, calculatedCameraPose);

            Matrix4 initialPose = calculatedCameraPose;

            // Note that here we only calculate the deltaFromReferenceFrame every 
            // DeltaFrameCalculationInterval frames to reduce computation time
            if (calculateDeltaFrame)
            {
                trackingSucceeded = FusionDepthProcessor.AlignPointClouds(
                    this.downsampledRaycastPointCloudFrame,
                    this.downsampledDepthPointCloudFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    this.downsampledDeltaFromReferenceFrameColorFrame,
                    ref calculatedCameraPose);

                this.UpsampleColorDeltasFrameNearestNeighbor(DownsampleFactor);

                this.UpdateAlignDeltas();

                // Set calculateDeltaFrame to false as we are rendering it here
                calculateDeltaFrame = false;
            }
            else
            {
                // Don't bother getting the residual delta from reference frame to cut computation time
                trackingSucceeded = FusionDepthProcessor.AlignPointClouds(
                    this.downsampledRaycastPointCloudFrame,
                    this.downsampledDepthPointCloudFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    null,
                    ref calculatedCameraPose);
            }

            if (trackingSucceeded)
            {
                bool failed = KinectFusionHelper.CameraTransformFailed(
                    initialPose,
                    calculatedCameraPose,
                    MaxTranslationDeltaAlignPointClouds,
                    MaxRotationDeltaAlignPointClouds);

                if (failed)
                {
                    trackingSucceeded = false;
                }
            }

            return trackingSucceeded;
        }

        /*------------------------------------------------------------------------------------------ */
        /// Track camera pose by aligning depth float image with reconstruction volume
        /// <param name="calculateDeltaFrame">Flag to calculate the delta frame.</param>
        /// <param name="calculatedCameraPos">The calculated camera position.</param>
        /// <returns>Returns true if tracking succeeded, false otherwise.</returns>
        private bool TrackCameraAlignDepthFloatToReconstruction(bool calculateDeltaFrame, ref Matrix4 calculatedCameraPos)
        {
            bool trackingSucceeded = false;

            // Note that here we only calculate the deltaFromReferenceFrame every 
            // DeltaFrameCalculationInterval frames to reduce computation time
            if (calculateDeltaFrame)
            {
                trackingSucceeded = this.volume.AlignDepthFloatToReconstruction(
                    this.depthFloatFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    this.deltaFromReferenceFrame,
                    out this.alignmentEnergy,
                    this.worldToCameraTransform);
            }
            else
            {
                // Don't bother getting the residual delta from reference frame to cut computation time
                trackingSucceeded = this.volume.AlignDepthFloatToReconstruction(
                    this.depthFloatFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    null,
                    out this.alignmentEnergy,
                    this.worldToCameraTransform);
            }

            if (!trackingSucceeded || this.alignmentEnergy > MaxAlignToReconstructionEnergyForSuccess || (this.alignmentEnergy <= MinAlignToReconstructionEnergyForSuccess && this.successfulFrameCount > 0))
            {
                trackingSucceeded = false;
            }
            else
            {
                // Tracking succeeded, get the updated camera pose
                calculatedCameraPos = this.volume.GetCurrentWorldToCameraTransform();
            }

            return trackingSucceeded;
        }

        /*------------------------------------------------------------------------------------------ */
        /// Set variables if camera tracking succeeded
        /// </summary>
        private void SetTrackingFailed()
        {
            // Clear successful frame count and increment the track error count
            this.trackingFailed = true;
            this.trackingHasFailedPreviously = true;
            this.trackingErrorCount++;
            this.successfulFrameCount = 0;
        }

        /*------------------------------------------------------------------------------------------ */
        /// Show low priority messages in the status bar. Low priority messages do not reset the fps counter,
        /// and will only be displayed when high priority message has at least StatusBarInterval seconds shown to the user.
        /// Messages that comes in burst or appear extremely frequently should be considered low priority.
        /// </summary>
        /// <param name="message">Message to show on status bar</param>
        private void ShowStatusMessageLowPriority(string message)
        {
            this.Dispatcher.BeginInvoke(
                (Action)(() =>
                {
                    // Make sure the high prioirty messages has at least StatusBarInterval seconds shown to the user.
                    if ((DateTime.Now - this.lastStatusTimestamp).Seconds >= StatusBarInterval)
                    {
                        this.statusBarText.Text = message;
						//PropertyChanged.Invoke( this, new PropertyChangedEventArgs(message) );
					}
                }));
        }

        /*------------------------------------------------------------------------------------------ */

        /// Perform camera pose finding when tracking is lost using AlignPointClouds.
        /// This is typically more successful than FindCameraPoseAlignDepthFloatToReconstruction.
        /// <returns>Returns true if a valid camera pose was found, otherwise false.</returns>
        private bool FindCameraPoseAlignPointClouds()
        {
            if (!this.cameraPoseFinderAvailable)
            {
                return false;
            }

			if ( this.CaptureColor )
			{
				// Process input color image to make it equal in size to the depth image
				this.ProcessColorForCameraPoseFinder();
			}
            
			MatchCandidates matchCandidates = this.cameraPoseFinder.FindCameraPose(
                this.depthFloatFrame,
                this.resampledColorFrame);

            if (null == matchCandidates)
            {
                return false;
            }

            int poseCount = matchCandidates.GetPoseCount();
            float minDistance = matchCandidates.CalculateMinimumDistance();

            if (0 == poseCount || minDistance >= CameraPoseFinderDistanceThresholdReject)
            {
                //this.ShowStatusMessage(Properties.Resources.PoseFinderNotEnoughMatches);
                this.ShowStatusMessage(": FindCameraPose exited early as not good enough pose matches."); 
                return false;
            }

            // Smooth the depth frame
            this.volume.SmoothDepthFloatFrame(this.depthFloatFrame, this.smoothDepthFloatFrame, SmoothingKernelWidth, SmoothingDistanceThreshold);

            // Calculate point cloud from the smoothed frame
            FusionDepthProcessor.DepthFloatFrameToPointCloud(this.smoothDepthFloatFrame, this.depthPointCloudFrame);

            double smallestEnergy = double.MaxValue;
            int smallestEnergyNeighborIndex = -1;

            int bestNeighborIndex = -1;
            Matrix4 bestNeighborCameraPose = Matrix4.Identity;

            double bestNeighborAlignmentEnergy = MaxAlignPointCloudsEnergyForSuccess;

            // Run alignment with best matched poseCount (i.e. k nearest neighbors (kNN))
            int maxTests = Math.Min(MaxCameraPoseFinderPoseTests, poseCount);

            var neighbors = matchCandidates.GetMatchPoses();

            for (int n = 0; n < maxTests; n++)
            {
                // Run the camera tracking algorithm with the volume
                // this uses the raycast frame and pose to find a valid camera pose by matching the raycast against the input point cloud
                Matrix4 poseProposal = neighbors[n];

                // Get the saved pose view by raycasting the volume
                this.volume.CalculatePointCloud(this.raycastPointCloudFrame, poseProposal);

                bool success = this.volume.AlignPointClouds(
                    this.raycastPointCloudFrame,
                    this.depthPointCloudFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    this.resampledColorFrame,
                    out this.alignmentEnergy,
                    ref poseProposal);

                bool relocSuccess = success && this.alignmentEnergy < bestNeighborAlignmentEnergy && this.alignmentEnergy > MinAlignPointCloudsEnergyForSuccess;

                if (relocSuccess)
                {
                    bestNeighborAlignmentEnergy = this.alignmentEnergy;
                    bestNeighborIndex = n;

                    // This is after tracking succeeds, so should be a more accurate pose to store...
                    bestNeighborCameraPose = poseProposal;

                    // Update the delta image
                    this.resampledColorFrame.CopyPixelDataTo(this.deltaFromReferenceFramePixelsArgb);
                }

                // Find smallest energy neighbor independent of tracking success
                if (this.alignmentEnergy < smallestEnergy)
                {
                    smallestEnergy = this.alignmentEnergy;
                    smallestEnergyNeighborIndex = n;
                }
            }

            matchCandidates.Dispose();

            // Use the neighbor with the smallest residual alignment energy
            // At the cost of additional processing we could also use kNN+Mean camera pose finding here
            // by calculating the mean pose of the best n matched poses and also testing this to see if the 
            // residual alignment energy is less than with kNN.
            if (bestNeighborIndex > -1)
            {
                this.worldToCameraTransform = bestNeighborCameraPose;
                this.SetReferenceFrame(this.worldToCameraTransform);

                // Tracking succeeded!
                this.SetTrackingSucceeded();

                this.UpdateAlignDeltas();

                this.ShowStatusMessageLowPriority(": Camera Pose Finder SUCCESS! Residual energy= " + string.Format(CultureInfo.InvariantCulture, "{0:0.00000}", bestNeighborAlignmentEnergy) + ", " + poseCount + " frames stored, minimum distance=" + minDistance + ", best match index=" + bestNeighborIndex);

                return true;
            }
            else
            {
                this.worldToCameraTransform = neighbors[smallestEnergyNeighborIndex];
                this.SetReferenceFrame(this.worldToCameraTransform);

                // Camera pose finding failed - return the tracking failed error code
                this.SetTrackingFailed();

                // Tracking Failed will be set again on the next iteration in ProcessDepth
                this.ShowStatusMessageLowPriority(": Camera Pose Finder FAILED! Residual energy=" + string.Format(CultureInfo.InvariantCulture, "{0:0.00000}", smallestEnergy) + ", " + poseCount + " frames stored, minimum distance=" + minDistance + ", best match index=" + smallestEnergyNeighborIndex);

                return false;
            }
        }

        /*------------------------------------------------------------------------------------------ */

        /// Process input color image to make it equal in size to the depth image
        /// </summary>
        private unsafe void ProcessColorForCameraPoseFinder()
        {
            if (null == this.resampledColorFrame || null == this.downsampledDepthImagePixels)
            {
                throw new ArgumentException(": inputs null");
            }

            if (this.depthWidth != RawDepthWidth || this.depthHeight != RawDepthHeight
                || this.colorWidth != RawColorWidth || this.colorHeight != RawColorHeight)
            {
                return;
            }

            float factor = RawColorWidth / RawDepthHeightWithSpecialRatio;
            const int FilledZeroMargin = (RawDepthHeight - RawDepthHeightWithSpecialRatio) / 2;

            // Here we make use of unsafe code to just copy the whole pixel as an int for performance reasons, as we do
            // not need access to the individual rgba components.
            fixed (byte* ptrColorPixels = this.colorImagePixels)
            {
                int* rawColorPixels = (int*)ptrColorPixels;

                Parallel.For(
                    FilledZeroMargin,
                    this.depthHeight - FilledZeroMargin,
                    y =>
                    {
                        int destIndex = y * this.depthWidth;

                        for (int x = 0; x < this.depthWidth; ++x, ++destIndex)
                        {
                            int srcX = (int)(x * factor);
                            int srcY = (int)(y * factor);
                            int sourceColorIndex = (srcY * this.colorWidth) + srcX;

                            this.resampledColorImagePixels[destIndex] = rawColorPixels[sourceColorIndex];
                        }
                    });
            }

            this.resampledColorFrame.CopyPixelDataFrom(this.resampledColorImagePixels);
        }

        /*------------------------------------------------------------------------------------------ */
        /// Updates the align deltas image given the current mode
        /// </summary>
        private void UpdateAlignDeltas()
        {
			// IF Use Camera Pose Finder is checked
			if ( this.autoFindCameraPoseWhenLost)
            {
                //if (deltaFrameRef.IsSelected)
                {
                    Dispatcher.BeginInvoke(
                    (Action)
                    (() =>
                        this.RenderAlignDeltasColorImage(
                            ref this.deltaFromReferenceFrameBitmap,
                            this.xamlFrame_deltaFromReferenceImage)));
                }
            }
            else
            {
                //if (deltaFrameRef.IsSelected)
                {
                    Dispatcher.BeginInvoke(
                    (Action)
                    (() =>
                        this.RenderAlignDeltasFloatImage(
                            this.deltaFromReferenceFrame,
                            ref this.deltaFromReferenceFrameBitmap,
                            this.xamlFrame_deltaFromReferenceImage)));
                }
                
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Render Fusion AlignPointClouds color deltas frame to UI
        /// </summary>
        /// <param name="bitmap">Bitmap contains float frame data for rendering</param>
        /// <param name="image">UI image component to render float frame to</param>
        private void RenderAlignDeltasColorImage(ref WriteableBitmap bitmap, System.Windows.Controls.Image image)
        {
            if (null == bitmap || this.depthWidth != bitmap.Width || this.depthHeight != bitmap.Height)
            {
                // Create bitmap of correct format
                bitmap = new WriteableBitmap(this.depthWidth, this.depthHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set bitmap as source to UI image object
                image.Source = bitmap;
            }

            // Copy colored pixels to bitmap
            bitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthWidth, this.depthHeight),
                        this.deltaFromReferenceFramePixelsArgb,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }

        /*------------------------------------------------------------------------------------------ */
        /// Render Fusion AlignDepthFloatToReconstruction float deltas frame to UI
        /// </summary>
        /// <param name="alignDeltasFloatFrame">Fusion depth float frame</param>
        /// <param name="bitmap">Bitmap contains float frame data for rendering</param>
        /// <param name="image">UI image component to render float frame to</param>
        private void RenderAlignDeltasFloatImage(FusionFloatImageFrame alignDeltasFloatFrame, ref WriteableBitmap bitmap, System.Windows.Controls.Image image)
        {
            if (null == alignDeltasFloatFrame)
            {
                return;
            }

            if (null == bitmap || alignDeltasFloatFrame.Width != bitmap.Width || alignDeltasFloatFrame.Height != bitmap.Height)
            {
                // Create bitmap of correct format
                bitmap = new WriteableBitmap(alignDeltasFloatFrame.Width, alignDeltasFloatFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set bitmap as source to UI image object
                image.Source = bitmap;
            }

            alignDeltasFloatFrame.CopyPixelDataTo(this.deltaFromReferenceFrameFloatPixels);

            Parallel.For(
            0,
            alignDeltasFloatFrame.Height,
            y =>
            {
                int index = y * alignDeltasFloatFrame.Width;
                for (int x = 0; x < alignDeltasFloatFrame.Width; ++x, ++index)
                {
                    float residue = this.deltaFromReferenceFrameFloatPixels[index];

                    if (residue < 1.0f)
                    {
                        this.deltaFromReferenceFramePixelsArgb[index] = (byte)(255.0f * KinectFusionHelper.ClampFloatingPoint(1.0f - residue, 0.0f, 1.0f)); // blue
                        this.deltaFromReferenceFramePixelsArgb[index] |= ((byte)(255.0f * KinectFusionHelper.ClampFloatingPoint(1.0f - Math.Abs(residue), 0.0f, 1.0f))) << 8; // green
                        this.deltaFromReferenceFramePixelsArgb[index] |= ((byte)(255.0f * KinectFusionHelper.ClampFloatingPoint(1.0f + residue, 0.0f, 1.0f))) << 16; // red
                    }
                    else
                    {
                        this.deltaFromReferenceFramePixelsArgb[index] = 0;
                    }
                }
            });

            // Copy colored pixels to bitmap
            bitmap.WritePixels(
                        new Int32Rect(0, 0, alignDeltasFloatFrame.Width, alignDeltasFloatFrame.Height),
                        this.deltaFromReferenceFramePixelsArgb,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }
        /*------------------------------------------------------------------------------------------ */
        /// This is used to set the reference frame.
        /// </summary>
        /// <param name="pose">The pose to use.</param>
        private void SetReferenceFrame(Matrix4 pose)
        {
            // Get the saved pose view by raycasting the volume
            this.volume.CalculatePointCloudAndDepth(this.raycastPointCloudFrame, this.smoothDepthFloatFrame, null, pose);

            // Set this as the reference frame for the next call to AlignDepthFloatToReconstruction
            this.volume.SetAlignDepthFloatToReconstructionReferenceFrame(this.smoothDepthFloatFrame);
        }

        /*------------------------------------------------------------------------------------------ */
        /// Set variables if camera tracking succeeded
        /// </summary>
        private void SetTrackingSucceeded()
        {
            // Clear track error count and increment the successful frame count
            this.trackingFailed = false;
            this.trackingErrorCount = 0;
            this.successfulFrameCount++;
        }

        /*------------------------------------------------------------------------------------------ */
        /// Up sample color delta from reference frame with nearest neighbor - replicates pixels
        /// </summary>
        /// <param name="factor">The up sample factor (2=x*2,y*2, 4=x*4,y*4, 8=x*8,y*8, 16=x*16,y*16).</param>
        private unsafe void UpsampleColorDeltasFrameNearestNeighbor(int factor)
        {
            if (null == this.downsampledDeltaFromReferenceFrameColorFrame || null == this.downsampledDeltaFromReferenceColorPixels || null == this.deltaFromReferenceFramePixelsArgb)
            {
                throw new ArgumentException("inputs null");
            }

            if (false == (2 == factor || 4 == factor || 8 == factor || 16 == factor))
            {
                throw new ArgumentException("factor != 2, 4, 8 or 16");
            }

            int upsampleWidth = this.downsampledWidth * factor;
            int upsampleHeight = this.downsampledHeight * factor;

            if (this.depthWidth != upsampleWidth || this.depthHeight != upsampleHeight)
            {
                throw new ArgumentException("upsampled image size != depth size");
            }

            this.downsampledDeltaFromReferenceFrameColorFrame.CopyPixelDataTo(this.downsampledDeltaFromReferenceColorPixels);

            // Here we make use of unsafe code to just copy the whole pixel as an int for performance reasons, as we do
            // not need access to the individual rgba components.
            fixed (int* rawColorPixelPtr = this.downsampledDeltaFromReferenceColorPixels)
            {
                int* rawColorPixels = (int*)rawColorPixelPtr;

                // Note we run this only for the source image height pixels to sparsely fill the destination with rows
                Parallel.For(
                    0,
                    this.downsampledHeight,
                    y =>
                    {
                        int destIndex = y * upsampleWidth * factor;
                        int sourceColorIndex = y * this.downsampledWidth;

                        for (int x = 0; x < this.downsampledWidth; ++x, ++sourceColorIndex)
                        {
                            int color = rawColorPixels[sourceColorIndex];

                            // Replicate pixels horizontally
                            for (int colFactorIndex = 0; colFactorIndex < factor; ++colFactorIndex, ++destIndex)
                            {
                                // Replicate pixels vertically
                                for (int rowFactorIndex = 0; rowFactorIndex < factor; ++rowFactorIndex)
                                {
                                    // Copy color pixel
                                    this.deltaFromReferenceFramePixelsArgb[destIndex + (rowFactorIndex * upsampleWidth)] = color;
                                }
                            }
                        }
                    });
            }

            int sizeOfInt = sizeof(int);
            int rowByteSize = this.downsampledHeight * sizeOfInt;

            // Duplicate the remaining rows with memcpy
            for (int y = 0; y < this.downsampledHeight; ++y)
            {
                // iterate only for the smaller number of rows
                int srcRowIndex = upsampleWidth * factor * y;

                // Duplicate lines
                for (int r = 1; r < factor; ++r)
                {
                    int index = upsampleWidth * ((y * factor) + r);

                    System.Buffer.BlockCopy(
                        this.deltaFromReferenceFramePixelsArgb, srcRowIndex * sizeOfInt, this.deltaFromReferenceFramePixelsArgb, index * sizeOfInt, rowByteSize);
                }
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Update the virtual camera transform from this process.
        /// </summary>
        private void UpdateVirtualCameraTransform()
        {
            // Update just the virtual camera pose from the tracked camera
            // We do not update the frustum here, as we do not render it when in Kinect camera view.
            this.virtualCamera.WorldToCameraMatrix4 = this.worldToCameraTransform;
        }

        /*------------------------------------------------------------------------------------------ */
        /// Downsample depth pixels with nearest neighbor
        /// </summary>
        /// <param name="dest">The destination depth image.</param>
        /// <param name="factor">The downsample factor (2=x/2,y/2, 4=x/4,y/4, 8=x/8,y/8, 16=x/16,y/16).</param>
        private unsafe void DownsampleDepthFrameNearestNeighbor(FusionFloatImageFrame dest, int factor)
        {
            if (null == dest || null == this.downsampledDepthImagePixels)
            {
                throw new ArgumentException("inputs null");
            }

            if (false == (2 == factor || 4 == factor || 8 == factor || 16 == factor))
            {
                throw new ArgumentException("factor != 2, 4, 8 or 16");
            }

            int downsampleWidth = this.depthWidth / factor;
            int downsampleHeight = this.depthHeight / factor;

            if (dest.Width != downsampleWidth || dest.Height != downsampleHeight)
            {
                throw new ArgumentException("dest != downsampled image size");
            }

            if (settingsWindow.mirrorDepth == true)
            {
                fixed (ushort* rawDepthPixelPtr = this.depthImagePixels)
                {
                    ushort* rawDepthPixels = (ushort*)rawDepthPixelPtr;

                    Parallel.For(
                        0,
                        downsampleHeight,
                        y =>
                        {
                            int destIndex = y * downsampleWidth;
                            int sourceIndex = y * this.depthWidth * factor;

                            for (int x = 0; x < downsampleWidth; ++x, ++destIndex, sourceIndex += factor)
                            {
                                // Copy depth value
                                this.downsampledDepthImagePixels[destIndex] = (float)rawDepthPixels[sourceIndex] * 0.001f;
                            }
                        });
                }
            }
            else
            {
                fixed (ushort* rawDepthPixelPtr = this.depthImagePixels)
                {
                    ushort* rawDepthPixels = (ushort*)rawDepthPixelPtr;

                    // Horizontal flip the color image as the standard depth image is flipped internally in Kinect Fusion
                    // to give a viewpoint as though from behind the Kinect looking forward by default.
                    Parallel.For(
                        0,
                        downsampleHeight,
                        y =>
                        {
							int destIndex = y * downsampleWidth;
							int flippedDestIndex = (y * downsampleWidth) + (downsampleWidth - 1);
                            int sourceIndex = y * this.depthWidth * factor;

                            for (int x = 0; x < downsampleWidth; ++x, --flippedDestIndex, sourceIndex += factor)
                            {
                                // Copy depth value
                                this.downsampledDepthImagePixels[flippedDestIndex] = (float)rawDepthPixels[sourceIndex] * 0.001f;
                            }
                        });
                }
            }

            dest.CopyPixelDataFrom(this.downsampledDepthImagePixels);
        }

        /*------------------------------------------------------------------------------------------ */
        /// Perform volume depth data integration
        /// </summary>
        /// <returns>Returns true if a color frame is available for further processing, false otherwise.</returns>
        private bool IntegrateData()
        {
            // Color may opportunistically be available here - check
            bool colorAvailable = this.colorReadyEvent.WaitOne(0);

            // Don't integrate depth data into the volume if:
            // 1) tracking failed
            // 2) camera pose finder is off and we have paused capture
            // 3) camera pose finder is on and we are still under the m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure
            //    number of successful frames count.
            bool integrateData = !this.trackingFailed && !this.PauseIntegration &&
                (!this.cameraPoseFinderAvailable || (this.cameraPoseFinderAvailable && !(this.trackingHasFailedPreviously && this.successfulFrameCount < MinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure)));

            // Integrate the frame to volume
            if (integrateData)
            {
                bool integrateColor = this.processedFrameCount % ColorIntegrationInterval == 0 && colorAvailable;

                // Reset this flag as we are now integrating data again
                this.trackingHasFailedPreviously = false;

                if (this.captureColor && integrateColor)
                {
                    // Pre-process color
                    this.MapColorToDepth();

                    // Integrate color and depth
                    this.volume.IntegrateFrame(
                        this.depthFloatFrame,
                        this.resampledColorFrameDepthAligned,
                        this.integrationWeight,
                        FusionDepthProcessor.DefaultColorIntegrationOfAllAngles,
                        this.worldToCameraTransform);

                    // Flag that we have captured color
                    this.colorCaptured = true;
                }
                else
                {
                    // Just integrate depth
                    this.volume.IntegrateFrame(
                        this.depthFloatFrame,
                        this.integrationWeight,
                        this.worldToCameraTransform);
                }

                // Reset color ready event
                this.colorReadyEvent.Reset();
            }

            return colorAvailable;
        }

        /*------------------------------------------------------------------------------------------ */
        /// Process the color and depth inputs, converting the color into the depth space
        /// </summary>
        private unsafe void MapColorToDepth()
        {
            this.mapper.MapDepthFrameToColorSpace(this.depthImagePixels, this.colorCoordinates);

            lock (this.rawDataLock)
            {
                // Fill in the visibility depth map.
                Array.Clear(this.depthVisibilityTestMap, 0, this.depthVisibilityTestMap.Length);
                fixed (ushort* ptrDepthVisibilityPixels = this.depthVisibilityTestMap, ptrDepthPixels = this.depthImagePixels)
                {
                    for (int index = 0; index < this.depthImagePixels.Length; ++index)
                    {
                        if (!float.IsInfinity(this.colorCoordinates[index].X) && !float.IsInfinity(this.colorCoordinates[index].Y))
                        {
                            int x = (int)(Math.Floor(this.colorCoordinates[index].X + 0.5f) / ColorDownsampleFactor);
                            int y = (int)(Math.Floor(this.colorCoordinates[index].Y + 0.5f) / ColorDownsampleFactor);

                            if ((x >= 0) && (x < this.depthVisibilityTestMapWidth) &&
                                (y >= 0) && (y < this.depthVisibilityTestMapHeight))
                            {
                                int depthVisibilityTestIndex = (y * this.depthVisibilityTestMapWidth) + x;
                                if ((ptrDepthVisibilityPixels[depthVisibilityTestIndex] == 0) ||
                                    (ptrDepthVisibilityPixels[depthVisibilityTestIndex] > ptrDepthPixels[index]))
                                {
                                    ptrDepthVisibilityPixels[depthVisibilityTestIndex] = ptrDepthPixels[index];
                                }
                            }
                        }
                    }
                }

                if (this.mirrorDepth)
                {
                    // Here we make use of unsafe code to just copy the whole pixel as an int for performance reasons, as we do
                    // not need access to the individual rgba components.
                    fixed (byte* ptrColorPixels = this.colorImagePixels)
                    {
                        int* rawColorPixels = (int*)ptrColorPixels;

                        Parallel.For(
                            0,
                            this.depthHeight,
                            y =>
                            {
                                int destIndex = y * this.depthWidth;

                                for (int x = 0; x < this.depthWidth; ++x, ++destIndex)
                                {
                                    // calculate index into depth array
                                    int colorInDepthX = (int)Math.Floor(colorCoordinates[destIndex].X + 0.5);
                                    int colorInDepthY = (int)Math.Floor(colorCoordinates[destIndex].Y + 0.5);
                                    int depthVisibilityTestX = (int)(colorInDepthX / ColorDownsampleFactor);
                                    int depthVisibilityTestY = (int)(colorInDepthY / ColorDownsampleFactor);
                                    int depthVisibilityTestIndex = (depthVisibilityTestY * this.depthVisibilityTestMapWidth) + depthVisibilityTestX;

                                    // make sure the depth pixel maps to a valid point in color space
                                    if (colorInDepthX >= 0 && colorInDepthX < this.colorWidth && colorInDepthY >= 0
                                        && colorInDepthY < this.colorHeight && this.depthImagePixels[destIndex] != 0)
                                    {
                                        ushort depthTestValue = this.depthVisibilityTestMap[depthVisibilityTestIndex];

                                        if ((this.depthImagePixels[destIndex] - depthTestValue) < DepthVisibilityTestThreshold)
                                        {
                                            // Calculate index into color array
                                            int sourceColorIndex = colorInDepthX + (colorInDepthY * this.colorWidth);

                                            // Copy color pixel
                                            this.resampledColorImagePixelsAlignedToDepth[destIndex] = rawColorPixels[sourceColorIndex];
                                        }
                                        else
                                        {
                                            this.resampledColorImagePixelsAlignedToDepth[destIndex] = 0;
                                        }
                                    }
                                    else
                                    {
                                        this.resampledColorImagePixelsAlignedToDepth[destIndex] = 0;
                                    }
                                }
                            });
                    }
                }
                else
                {
                    // Here we make use of unsafe code to just copy the whole pixel as an int for performance reasons, as we do
                    // not need access to the individual rgba components.
                    fixed (byte* ptrColorPixels = this.colorImagePixels)
                    {
                        int* rawColorPixels = (int*)ptrColorPixels;

                        // Horizontal flip the color image as the standard depth image is flipped internally in Kinect Fusion
                        // to give a viewpoint as though from behind the Kinect looking forward by default.
                        Parallel.For(
                            0,
                            this.depthHeight,
                            y =>
                            {
                                int destIndex = y * this.depthWidth;
                                int flippedDestIndex = destIndex + (this.depthWidth - 1); // horizontally mirrored

                                for (int x = 0; x < this.depthWidth; ++x, ++destIndex, --flippedDestIndex)
                                {
                                    // calculate index into depth array
                                    int colorInDepthX = (int)Math.Floor(colorCoordinates[destIndex].X + 0.5);
                                    int colorInDepthY = (int)Math.Floor(colorCoordinates[destIndex].Y + 0.5);
                                    int depthVisibilityTestX = (int)(colorInDepthX / ColorDownsampleFactor);
                                    int depthVisibilityTestY = (int)(colorInDepthY / ColorDownsampleFactor);
                                    int depthVisibilityTestIndex = (depthVisibilityTestY * this.depthVisibilityTestMapWidth) + depthVisibilityTestX;

                                    // make sure the depth pixel maps to a valid point in color space
                                    if (colorInDepthX >= 0 && colorInDepthX < this.colorWidth && colorInDepthY >= 0
                                        && colorInDepthY < this.colorHeight && this.depthImagePixels[destIndex] != 0)
                                    {
                                        ushort depthTestValue = this.depthVisibilityTestMap[depthVisibilityTestIndex];

                                        if ((this.depthImagePixels[destIndex] - depthTestValue) < DepthVisibilityTestThreshold)
                                        {
                                            // Calculate index into color array- this will perform a horizontal flip as well
                                            int sourceColorIndex = colorInDepthX + (colorInDepthY * this.colorWidth);

                                            // Copy color pixel
                                            this.resampledColorImagePixelsAlignedToDepth[flippedDestIndex] = rawColorPixels[sourceColorIndex];
                                        }
                                        else
                                        {
                                            this.resampledColorImagePixelsAlignedToDepth[flippedDestIndex] = 0;
                                        }
                                    }
                                    else
                                    {
                                        this.resampledColorImagePixelsAlignedToDepth[flippedDestIndex] = 0;
                                    }
                                }
                            });
                    }
                }
            }

            this.resampledColorFrameDepthAligned.CopyPixelDataFrom(this.resampledColorImagePixelsAlignedToDepth);
        }

        /*------------------------------------------------------------------------------------------ */
        /// Render the reconstruction
        /// </summary>
        private void RenderReconstruction()
        {
			//Matrix4 cameraView;

			if (null == this.volume || this.savingMesh || null == this.raycastPointCloudFrame
                || null == this.shadedSurfaceFrame || null == this.shadedSurfaceNormalsFrame)
            {
                return;
            }

			// If KinectView option has been set, use the worldToCameraTransform, else use the virtualCamera transform
			Matrix4 cameraView = this.KinectView ? this.worldToCameraTransform : this.virtualCameraWorldToCameraMatrix4;

            if (this.captureColor)
            {
                this.volume.CalculatePointCloud(this.raycastPointCloudFrame, this.shadedSurfaceFrame, cameraView);
            }
            else
            {
                this.volume.CalculatePointCloud(this.raycastPointCloudFrame, cameraView);

                // Shade point cloud frame for rendering
                FusionDepthProcessor.ShadePointCloud(
                    this.raycastPointCloudFrame,
                    cameraView,
                    this.worldToBGRTransform,
                    this.displayNormals ? null : this.shadedSurfaceFrame,
                    this.displayNormals ? this.shadedSurfaceNormalsFrame : null);
            }

            // Update the rendered UI image
            Dispatcher.BeginInvoke((Action)(() => this.ReconstructFrameComplete()));

            this.lastRenderTimestamp = DateTime.UtcNow;
        }
        /*------------------------------------------------------------------------------------------ */
        private void ReconstructFrameComplete()
        {
            // Render shaded surface frame or shaded surface normals frame
            RenderColorImage(
                this.captureColor ? this.shadedSurfaceFrame : (this.displayNormals ? this.shadedSurfaceNormalsFrame : this.shadedSurfaceFrame),
                ref this.shadedSurfaceFramePixelsArgb,
                ref this.shadedSurfaceFrameBitmap,
                this.shadedSurfaceImage);
        }
        /*------------------------------------------------------------------------------------------ */
        /// Render Fusion color frame to UI
        /// </summary>
        /// <param name="colorFrame">Fusion color frame</param>
        /// <param name="colorPixels">Pixel buffer for fusion color frame</param>
        /// <param name="bitmap">Bitmap contains color frame data for rendering</param>
        /// <param name="image">UI image component to render the color frame</param>
        private static void RenderColorImage(FusionColorImageFrame colorFrame, ref int[] colorPixels, ref WriteableBitmap bitmap, System.Windows.Controls.Image image)
        {
            if (null == image || null == colorFrame)
            {
                return;
            }

            if (null == colorPixels || colorFrame.PixelDataLength != colorPixels.Length)
            {
                // Create pixel array of correct format
                colorPixels = new int[colorFrame.PixelDataLength];
            }

            if (null == bitmap || colorFrame.Width != bitmap.Width || colorFrame.Height != bitmap.Height)
            {
                // Create bitmap of correct format
                bitmap = new WriteableBitmap(colorFrame.Width, colorFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set bitmap as source to UI image object
                image.Source = bitmap;
            }

            // Copy pixel data to pixel buffer
            colorFrame.CopyPixelDataTo(colorPixels);

            // Write pixels to bitmap
            bitmap.WritePixels(
                        new Int32Rect(0, 0, colorFrame.Width, colorFrame.Height),
                        colorPixels,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }

        /*------------------------------------------------------------------------------------------ */
        /// Gets or sets a value indicating whether enable "Kinect View".
        /// </summary>
        public bool KinectView
        {
            get
            {
                return this.kinectView;
            }

            set
            {
				this.kinectView = value;

				if ( null != this.PropertyChanged )
				{
					this.PropertyChanged.Invoke( this, new PropertyChangedEventArgs( "KinectView" ) );
				}

				//Decide whether render the volume cube

				if ( this.kinectView == true )
				{
					this.virtualCamera.CameraTransformationChanged -= this.OnVirtualCameraTransformationChanged;
					this.virtualCamera.Detach( this.shadedSurfaceImage );

					if ( this.volumeGraphics )
					{
						// Do not render the frustum when in Kinect camera view with volume graphics active
						this.virtualCamera.RemoveFrustum3DGraphics();
					}
				}
				else
				{
					this.virtualCamera.Attach( this.shadedSurfaceImage );
					this.virtualCamera.CameraTransformationChanged += this.OnVirtualCameraTransformationChanged;

					if ( this.volumeGraphics )
					{
						// Re-render the frustum if we exit the Kinect camera view with volume graphics active
						this.virtualCamera.AddFrustum3DGraphics();
					}

					// Reset the virtual camera
					this.virtualCamera.Reset();
				}

				this.viewChanged = true;

                this.GraphicsViewport.InvalidateVisual();
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Event raised when the mouse updates the graphics camera transformation for the virtual camera
        /// Here we set the viewChanged flag to true, to cause a volume render when the WPF composite update event occurs
        /// </summary>
        /// <param name="sender">Event generator</param>
        /// <param name="e">Event parameter</param>
        private void OnVirtualCameraTransformationChanged(object sender, EventArgs e)
        {
            // Update the stored virtual camera pose
            this.virtualCameraWorldToCameraMatrix4 = this.virtualCamera.WorldToCameraMatrix4;
            this.viewChanged = true;
        }
        /*------------------------------------------------------------------------------------------ */

        /// Update the camera pose finder data.
        /// </summary>
        private void UpdateCameraPoseFinder()
        {
            if (null == this.depthFloatFrame || null == this.resampledColorFrame || null == this.cameraPoseFinder)
            {
                return;
            }

			if ( this.CaptureColor )
			{
				this.ProcessColorForCameraPoseFinder();
			}

            bool poseHistoryTrimmed = false;
            bool addedPose = false;

            // This function will add the pose to the camera pose finding database when the input frame's minimum
            // distance to the existing database is equal to or above CameraPoseFinderDistanceThresholdAccept 
            // (i.e. indicating that the input has become dis-similar to the existing database and a new frame 
            // should be captured). Note that the color and depth frames must be the same size, however, the 
            // horizontal mirroring setting does not have to be consistent between depth and color. It does have
            // to be consistent between camera pose finder database creation and calling FindCameraPose though,
            // hence we always reset both the reconstruction and database when changing the mirror depth setting.
            this.cameraPoseFinder.ProcessFrame(
                this.depthFloatFrame,
                this.resampledColorFrame,
                this.worldToCameraTransform,
                CameraPoseFinderDistanceThresholdAccept,
                out addedPose,
                out poseHistoryTrimmed);

            if (true == addedPose)
            {
                this.ShowStatusMessageLowPriority("Camera Pose Finder Added Frame! " + this.cameraPoseFinder.GetStoredPoseCount() + " frames stored, minimum distance >= " + CameraPoseFinderDistanceThresholdAccept);
            }

            if (true == poseHistoryTrimmed)
            {
                this.ShowStatusMessage("Kinect Fusion Camera Pose Finder pose history is full, overwritten oldest pose to store current pose."); 
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Handler for FPS timer tick
        /// </summary>
        /// <param name="sender">Object sending the event</param>
        /// <param name="e">Event arguments</param>
        private void FpsTimerTick(object sender, EventArgs e)
        {
            if (!this.savingMesh)
            {
                if (null == this.myKinectv2)
                {
                    // Show "No ready Kinect found!" on status bar
                    this.statusBarText.Text = "NoReadyKinect";
                    debugLog("No Kinect Available");
                }
                else
                {
                    // Calculate time span from last calculation of FPS
                    double intervalSeconds = (DateTime.UtcNow - this.lastFPSTimestamp).TotalSeconds;

                    // Calculate and show fps on status bar
                    this.fpsText.Text = "FPS: :";
                    this.fpsText.Text += ((double)this.processedFrameCount / intervalSeconds);
                }
            }

            // Reset frame counter
            this.processedFrameCount = 0;
            this.lastFPSTimestamp = DateTime.UtcNow;
        }

        /*------------------------------------------------------------------------------------------ */

        /// Handler for status bar timer tick
        /// </summary>
        /// <param name="sender">Object sending the event</param>
        /// <param name="e">Event arguments</param>
        private void StatusBarTimerTick(object sender, EventArgs e)
        {
            if (this.statusMessageQueue.Count > 0)
            {
                this.statusBarText.Text = this.statusMessageQueue.Dequeue();
				//debugLog( this.statusMessageQueue.Dequeue() );

                // Update the last timestamp of status message
                this.lastStatusTimestamp = DateTime.Now;
            }
        }
        /*------------------------------------------------------------------------------------------ */
        /// Allocate the frame buffers and memory used in the process for Kinect Fusion
        /// </summary>
        private void AllocateKinectFusionResources()
        {
            this.SafeDisposeFusionResources();

            // Allocate depth float frame
            this.depthFloatFrame = new FusionFloatImageFrame(this.depthWidth, this.depthHeight);

            // Allocate color frame for color data from Kinect mapped into depth frame
            this.resampledColorFrameDepthAligned = new FusionColorImageFrame(this.depthWidth, this.depthHeight);

            // Allocate delta from reference frame
            this.deltaFromReferenceFrame = new FusionFloatImageFrame(this.depthWidth, this.depthHeight);

            // Allocate point cloud frame
            this.raycastPointCloudFrame = new FusionPointCloudImageFrame(this.depthWidth, this.depthHeight);

            // Allocate shaded surface frame
            this.shadedSurfaceFrame = new FusionColorImageFrame(this.depthWidth, this.depthHeight);

            // Allocate shaded surface normals frame
            this.shadedSurfaceNormalsFrame = new FusionColorImageFrame(this.depthWidth, this.depthHeight);

            // Allocate re-sampled color at depth image size
            this.resampledColorFrame = new FusionColorImageFrame(this.depthWidth, this.depthHeight);

            // Allocate point cloud frame created from input depth
            this.depthPointCloudFrame = new FusionPointCloudImageFrame(this.depthWidth, this.depthHeight);

            // Allocate smoothed depth float frame
            this.smoothDepthFloatFrame = new FusionFloatImageFrame(this.depthWidth, this.depthHeight);

            this.downsampledWidth = this.depthWidth / DownsampleFactor;
            this.downsampledHeight = this.depthHeight / DownsampleFactor;

            // Allocate downsampled image frames
            this.downsampledDepthFloatFrame = new FusionFloatImageFrame(this.downsampledWidth, this.downsampledHeight);

            this.downsampledSmoothDepthFloatFrame = new FusionFloatImageFrame(this.downsampledWidth, this.downsampledHeight);

            this.downsampledRaycastPointCloudFrame = new FusionPointCloudImageFrame(this.downsampledWidth, this.downsampledHeight);

            this.downsampledDepthPointCloudFrame = new FusionPointCloudImageFrame(this.downsampledWidth, this.downsampledHeight);

            this.downsampledDeltaFromReferenceFrameColorFrame = new FusionColorImageFrame(this.downsampledWidth, this.downsampledHeight);

            int depthImageSize = this.depthWidth * this.depthHeight;
            int colorImageByteSize = this.colorWidth * this.colorHeight * sizeof(int);
            int downsampledDepthImageSize = this.downsampledWidth * this.downsampledHeight;

            // Create local depth pixels buffer
            this.depthImagePixels = new ushort[depthImageSize];

            // Create local color pixels buffer
            this.colorImagePixels = new byte[colorImageByteSize];

            // Create local color pixels buffer re-sampled to depth size
            this.resampledColorImagePixels = new int[depthImageSize];

            // Create float pixel array
            this.depthFloatFrameDepthPixels = new float[depthImageSize];

            // Create float pixel array
            this.deltaFromReferenceFrameFloatPixels = new float[depthImageSize];

            // Create colored pixel array of correct format
            this.depthFloatFramePixelsArgb = new int[depthImageSize];

            // Create colored pixel array of correct format
            this.deltaFromReferenceFramePixelsArgb = new int[depthImageSize];

            // Allocate the depth-color mapping points
            this.colorCoordinates = new ColorSpacePoint[depthImageSize];

            // Allocate color points re-sampled to depth size mapped into depth frame of reference
            this.resampledColorImagePixelsAlignedToDepth = new int[depthImageSize];

            // Allocate downsampled image arrays
            this.downsampledDepthImagePixels = new float[downsampledDepthImageSize];

            this.downsampledDeltaFromReferenceColorPixels = new int[downsampledDepthImageSize];

            // Create a camera pose finder with default parameters
            CameraPoseFinderParameters cameraPoseFinderParams = CameraPoseFinderParameters.Defaults;
            this.cameraPoseFinder = CameraPoseFinder.FusionCreateCameraPoseFinder(cameraPoseFinderParams);
        }
        /*------------------------------------------------------------------------------------------ */
        /// Dispose fusion resources safely
        /// </summary>
        private void SafeDisposeFusionResources()
        {
            if (null != this.depthFloatFrame)
            {
                this.depthFloatFrame.Dispose();
            }

            if (null != this.resampledColorFrameDepthAligned)
            {
                this.resampledColorFrameDepthAligned.Dispose();
            }

            if (null != this.deltaFromReferenceFrame)
            {
                this.deltaFromReferenceFrame.Dispose();
            }

            if (null != this.raycastPointCloudFrame)
            {
                this.raycastPointCloudFrame.Dispose();
            }

            if (null != this.shadedSurfaceFrame)
            {
                this.shadedSurfaceFrame.Dispose();
            }

            if (null != this.shadedSurfaceNormalsFrame)
            {
                this.shadedSurfaceNormalsFrame.Dispose();
            }

            if (null != this.resampledColorFrame)
            {
                this.resampledColorFrame.Dispose();
            }

            if (null != this.depthPointCloudFrame)
            {
                this.depthPointCloudFrame.Dispose();
            }

            if (null != this.smoothDepthFloatFrame)
            {
                this.smoothDepthFloatFrame.Dispose();
            }

            if (null != this.downsampledDepthFloatFrame)
            {
                this.downsampledDepthFloatFrame.Dispose();
            }

            if (null != this.downsampledSmoothDepthFloatFrame)
            {
                this.downsampledSmoothDepthFloatFrame.Dispose();
            }

            if (null != this.downsampledRaycastPointCloudFrame)
            {
                this.downsampledRaycastPointCloudFrame.Dispose();
            }

            if (null != this.downsampledDepthPointCloudFrame)
            {
                this.downsampledDepthPointCloudFrame.Dispose();
            }

            if (null != this.downsampledDeltaFromReferenceFrameColorFrame)
            {
                this.downsampledDeltaFromReferenceFrameColorFrame.Dispose();
            }

            if (null != this.cameraPoseFinder)
            {
                this.cameraPoseFinder.Dispose();
            }
        }
        /*------------------------------------------------------------------------------------------ */
        /// Gets or sets a value indicating whether to show volume graphics.
        /// </summary>
        public bool VolumeGraphics
        {
            get
            {
                return this.volumeGraphics;
            }

            set
            {
                this.volumeGraphics = value;

                if (null != this.PropertyChanged)
                {
                    this.PropertyChanged.Invoke(settingsWindow, new PropertyChangedEventArgs("VolumeGraphics"));
                }

                if (this.volumeGraphics)
                {
                    // Add the graphics to the visual tree

                    // Create axis-aligned coordinate cross 3D graphics at the WPF3D/reconstruction world origin
                    // Red is the +X axis, Green is the +Y axis, Blue is the +Z axis in the WPF3D coordinate system
                    // Note that this coordinate cross shows the WPF3D graphics coordinate system
                    // (right hand, erect so +Y up and +X right, +Z out of screen), rather than the reconstruction 
                    // volume coordinate system (right hand, rotated so +Y is down and +X is right, +Z into screen ).
                    this.CreateAxisAlignedCoordinateCross3DGraphics(new Point3D(0, 0, 0), OriginCoordinateCrossAxisSize, LineThickness);

                    // Create volume cube 3D graphics in WPF3D. The front top left corner is the actual origin of the volume
                    // voxel coordinate system, and shown with an overlaid coordinate cross.
                    // Red is the +X axis, Green is the +Y axis, Blue is the +Z axis in the voxel coordinate system
                    this.CreateCube3DGraphics(volumeCubeLineColor, LineThickness, new Vector3D(0, 0, 0));

                    this.AddVolumeCube3DGraphics();
                    this.AddAxisAlignedCoordinateCross3DGraphics();

                    if (!this.kinectView)
                    {
                        // Do not render the frustum when in Kinect camera view with volume graphics active
                        this.virtualCamera.AddFrustum3DGraphics();
                    }

                    // Add callback which is called every time WPF renders
                    System.Windows.Media.CompositionTarget.Rendering += this.CompositionTargetRendering;
                }
                else
                {
                    // Remove the graphics from the visual tree
                    this.DisposeVolumeCube3DGraphics();
                    this.DisposeAxisAlignedCoordinateCross3DGraphics();

                    this.virtualCamera.RemoveFrustum3DGraphics();

                    // Remove callback which is called every time WPF renders
                    System.Windows.Media.CompositionTarget.Rendering -= this.CompositionTargetRendering;
                }

                this.viewChanged = true;

                this.GraphicsViewport.InvalidateVisual();
            }
        }
        /*------------------------------------------------------------------------------------------ */
        /// Create an axis-aligned coordinate cross for rendering in the WPF3D coordinate system. 
        /// Red is the +X axis, Green is the +Y axis, Blue is the +Z axis
        /// </summary>
        /// <param name="crossOrigin">The origin of the coordinate cross in world space.</param>
        /// <param name="axisSize">The size of the axis in m.</param>
        /// <param name="thickness">The thickness of the lines in screen pixels.</param>
        private void CreateAxisAlignedCoordinateCross3DGraphics(Point3D crossOrigin, float axisSize, int thickness)
        {
            this.axisX = new ScreenSpaceLines3D();

            this.axisX.Points = new Point3DCollection();
            this.axisX.Points.Add(crossOrigin);
            this.axisX.Points.Add(new Point3D(crossOrigin.X + axisSize, crossOrigin.Y, crossOrigin.Z));

            this.axisX.Thickness = 2;
            this.axisX.Color = System.Windows.Media.Color.FromArgb(200, 255, 0, 0);   // Red (X)

            this.axisY = new ScreenSpaceLines3D();

            this.axisY.Points = new Point3DCollection();
            this.axisY.Points.Add(crossOrigin);
            this.axisY.Points.Add(new Point3D(crossOrigin.X, crossOrigin.Y + axisSize, crossOrigin.Z));

            this.axisY.Thickness = 2;
            this.axisY.Color = System.Windows.Media.Color.FromArgb(200, 0, 255, 0);   // Green (Y)

            this.axisZ = new ScreenSpaceLines3D();

            this.axisZ.Points = new Point3DCollection();
            this.axisZ.Points.Add(crossOrigin);
            this.axisZ.Points.Add(new Point3D(crossOrigin.X, crossOrigin.Y, crossOrigin.Z + axisSize));

            this.axisZ.Thickness = thickness;
            this.axisZ.Color = System.Windows.Media.Color.FromArgb(200, 0, 0, 255);   // Blue (Z)
        }

        /*------------------------------------------------------------------------------------------ */
        /// Add the coordinate cross axes to the visual tree
        private void AddAxisAlignedCoordinateCross3DGraphics()
        {
            if (this.haveAddedCoordinateCross)
            {
                return;
            }

            if (null != this.axisX)
            {
                this.GraphicsViewport.Children.Add(this.axisX);

                this.haveAddedCoordinateCross = true;
            }

            if (null != this.axisY)
            {
                this.GraphicsViewport.Children.Add(this.axisY);
            }

            if (null != this.axisZ)
            {
                this.GraphicsViewport.Children.Add(this.axisZ);
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Called on each render of WPF (usually around 60Hz)
        /// </summary>
        /// <param name="sender">Object sending the event</param>
        /// <param name="e">Event arguments</param>
        private void CompositionTargetRendering(object sender, EventArgs e)
        {
            // If the viewChanged flag is used so we only raycast the volume when something changes
            // When reconstructing we call RenderReconstruction manually for every integrated depth frame (see ReconstructDepthData)
            if (this.viewChanged)
            {
                this.viewChanged = false;
                this.RenderReconstruction();
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Dispose the coordinate cross axes from the visual tree
        /// </summary>
        private void DisposeAxisAlignedCoordinateCross3DGraphics()
        {
            if (this.haveAddedCoordinateCross)
            {
                this.RemoveAxisAlignedCoordinateCross3DGraphics();
            }

            if (null != this.axisX)
            {
                this.axisX.Dispose();
                this.axisX = null;
            }

            if (null != this.axisY)
            {
                this.axisY.Dispose();
                this.axisY = null;
            }

            if (null != this.axisZ)
            {
                this.axisZ.Dispose();
                this.axisZ = null;
            }
        }

        /*------------------------------------------------------------------------------------------ */
        /// Remove the coordinate cross axes from the visual tree
        /// </summary>
        private void RemoveAxisAlignedCoordinateCross3DGraphics()
        {
            if (null != this.axisX)
            {
                this.GraphicsViewport.Children.Remove(this.axisX);
            }

            if (null != this.axisY)
            {
                this.GraphicsViewport.Children.Remove(this.axisY);
            }

            if (null != this.axisZ)
            {
                this.GraphicsViewport.Children.Remove(this.axisZ);
            }

            this.haveAddedCoordinateCross = false;
        }

		/*------------------------------------------------------------------------------------------ */
		/// Gets a value indicating whether rendering is overdue 
		/// (i.e. time interval since last render > RenderIntervalMilliseconds)
		/// </summary>
		public bool IsRenderOverdue
		{
			get
			{
				return (DateTime.UtcNow - this.lastRenderTimestamp).TotalMilliseconds >= RenderIntervalMilliseconds;
			}
		}

		/*------------------------------------------------------------------------------------------ */
		/// Handles the user clicking on the screenshot button
		/// </summary>
		/// <param name="sender">object sending the event</param>
		/// <param name="e">event arguments</param>
		private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            if (this.infraredBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.infraredBitmap));

                string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myDesktop= Environment.GetFolderPath(Environment.SpecialFolder.Desktop);

                string path = System.IO.Path.Combine(myDesktop, "KinectScreenshot-Infrared-" + time + ".png");

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                    //this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.SavedScreenshotStatusTextFormat, path);
                }
                catch (IOException)
                {
                    this.StatusText = string.Format("Failed to write screenshot");
                    debugLog("Failed to write screenshot");
                }
            }
        }
        
        /*------------------------------------------------------------------------------------------ */
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.infraredBitmap;
            }
        }

		/*------------------------------------------------------------------------------------------ */
		/// <summary>
		/// Gets or sets a value indicating whether to use the camera pose finder.
		/// </summary>
		public bool UseCameraPoseFinder
		{
			get
			{
				return this.autoFindCameraPoseWhenLost;
			}

			set
			{
				this.autoFindCameraPoseWhenLost = value;
				if ( null != this.PropertyChanged )
				{
					this.PropertyChanged.Invoke( this, new PropertyChangedEventArgs( "UseCameraPoseFinder" ) );
				}
			}
		}

		/// <summary>
		/// Handler for click event from "Reset Virtual Camera" button
		/// </summary>
		/// <param name="sender">Event sender</param>
		/// <param name="e">Event arguments</param>
		public void ResetCameraButtonClick( object sender, RoutedEventArgs e )
		{
			if ( null != this.virtualCamera )
			{
				this.virtualCamera.Reset();
				this.viewChanged = true;
			}
		}

		/// <summary>
		/// Handler for click event from "Reset Reconstruction" button
		/// </summary>
		/// <param name="sender">Event sender</param>
		/// <param name="e">Event arguments</param>
		public void ResetReconstructionButtonClick( object sender, RoutedEventArgs e )
		{
			if ( null == this.myKinectv2 )
			{
				return;
			}

			// Signal the worker thread to reset the volume
			this.resetReconstruction = true;

			// Update manual reset information to status bar
			//this.ShowStatusMessage( Properties.Resources.ResetVolume );
		}

		/// <summary>
		/// Gets or sets the integration weight.
		/// </summary>
		public double IntegrationWeight
		{
			get
			{
				return (double)this.integrationWeight;
			}

			set
			{
				this.integrationWeight = (short)(value + 0.5);
				if ( null != this.PropertyChanged )
				{
					this.PropertyChanged.Invoke( this, new PropertyChangedEventArgs( "IntegrationWeight" ) );
				}
			}
		}

		/// <summary>
		/// Gets or sets the voxels per meter value.
		/// </summary>
		public double VoxelsPerMeter
		{
			get
			{
				return (double)this.voxelsPerMeter;
			}

			set
			{
				this.voxelsPerMeter = (int)value;
				if ( null != this.PropertyChanged )
				{
					this.PropertyChanged.Invoke( this, new PropertyChangedEventArgs( "VoxelsPerMeter" ) );
				}
			}
		}

		/// <summary>
		/// Gets or sets the X-axis volume resolution.
		/// </summary>
		public double VoxelsX
		{
			get
			{
				return (double)this.voxelsX;
			}

			set
			{
				this.voxelsX = (int)(value + 0.5);
				if ( null != this.PropertyChanged )
				{
					this.PropertyChanged.Invoke( this, new PropertyChangedEventArgs( "VoxelsX" ) );
				}
			}
		}

		/// <summary>
		/// Gets or sets the Y-axis volume resolution.
		/// </summary>
		public double VoxelsY
		{
			get
			{
				return (double)this.voxelsY;
			}

			set
			{
				this.voxelsY = (int)(value + 0.5);
				if ( null != this.PropertyChanged )
				{
					this.PropertyChanged.Invoke( this, new PropertyChangedEventArgs( "VoxelsY" ) );
				}
			}
		}

		/// <summary>
		/// Gets or sets the Z-axis volume resolution.
		/// </summary>
		public double VoxelsZ
		{
			get
			{
				return (double)this.voxelsZ;
			}

			set
			{
				this.voxelsZ = (int)(value + 0.5);
				if ( null != this.PropertyChanged )
				{
					this.PropertyChanged.Invoke( this, new PropertyChangedEventArgs( "VoxelsZ" ) );
				}
			}
		}

		/*------------------------------------------------------------------------------------------ */
		/*------------------------------------------------------------------------------------------ */
		/*------------------------------------------------------------------------------------------ */
		/*------------------------------------------------------------------------------------------ */
		/*------------------------------------------------------------------------------------------ */
		/*------------------------------------------------------------------------------------------ */
		/*------------------------------------------------------------------------------------------ */
		/*------------------------------------------------------------------------------------------ */
		/*----------------------------------- END KINECT FUSION CODE MERGE-------------------------- */

		private void btnConnect_Click(object sender, RoutedEventArgs e)
        {
            int deviceMemoryKB = 0;

            // Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
            try
            {
                string deviceDescription = string.Empty;
                string deviceInstancePath = string.Empty;

                FusionDepthProcessor.GetDeviceInfo(ProcessorType, DeviceToUse, out deviceDescription, out deviceInstancePath, out deviceMemoryKB);
            }
            catch (IndexOutOfRangeException)
            {
                // Thrown when index is out of range for processor type or there is no DirectX11 capable device installed.
                // As we set -1 (auto-select default) for the DeviceToUse above, this indicates that there is no DirectX11 
                // capable device. The options for users in this case are to either install a DirectX11 capable device 
                // (see documentation for recommended GPUs) or to switch to non-real-time CPU based reconstruction by 
                // changing ProcessorType to ReconstructionProcessor.Cpu
                //this.statusBarText.Text = Properties.Resources.NoDirectX11CompatibleDeviceOrInvalidDeviceIndex;
                this.statusBarText.Text = "No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction.";
                debugLog("No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction.");
                return;
            }
            catch (DllNotFoundException)
            {
                //this.statusBarText.Text = Properties.Resources.MissingPrerequisite;
                this.statusBarText.Text = "A prerequisite component for Kinect Fusion is missing. Please refer to the Toolkit documentation for assistance";
                debugLog("A prerequisite component for Kinect Fusion is missing. Please refer to the Toolkit documentation for assistance");
                return;
            }
            catch (InvalidOperationException ex)
            {
                this.statusBarText.Text = ex.Message;
                debugLog(ex.Message);
                statsWindow.scrollViewer.ScrollToEnd();
                return;
            }

            //this.settingsWindow.VoxelsXSlider.Maximum = 512;
            //this.settingsWindow.VoxelsYSlider.Maximum = 512;
            //this.settingsWindow.VoxelsZSlider.Maximum = 512;

            settingsWindow.VoxelsXSlider.Value = voxelsX;
            settingsWindow.VoxelsYSlider.Value = voxelsY;
            settingsWindow.VoxelsZSlider.Value = voxelsZ;
            settingsWindow.VoxelsPerMeterSlider.Value = voxelsPerMeter;

            ////voxelsX = this.settingsWindow.VoxelsXSlider.Value
            //this.settingsWindow.VoxelsXSlider.ValueChanged += VoxelsXSlider_ValueChanged;


            // display on logging window
            debugLog(" Trying to Connecting to Kinect Sensor ");

            // One sensor is supported
            myKinectv2 = KinectSensor.GetDefault();

            if (myKinectv2 == null)
            {
                //this.statusBarText.Text = Properties.Resources.NoReadyKinect;
                this.statusBarText.Text = "No ready Kinect found!";
                debugLog(" <!-- No ready Kinect found! --!>");
                return;
            }
            else if (myKinectv2 != null)
            {

				debugLog( " Kinect Found - Initializing device " );

				// get the coordinate mapper
				this.mapper = this.myKinectv2.CoordinateMapper;

				// open the sensor
				myKinectv2.Open();

				//Specifying which streams are required
				_reader = myKinectv2.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared);

                //// open the reader for frames
                //this._colorFrameReader = this.myKinectv2.ColorFrameSource.OpenReader();
                //this._depthFrameReader = this.myKinectv2.DepthFrameSource.OpenReader();
                //this._infraredFrameReader = this.myKinectv2.InfraredFrameSource.OpenReader();

                //this.colorFrameDescription = this.myKinectv2.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
                this.colorFrameDescription = this.myKinectv2.ColorFrameSource.FrameDescription;
                this.colorWidth = colorFrameDescription.Width;
                this.colorHeight = colorFrameDescription.Height;
                this.colorPixelCount = this.colorWidth * this.colorHeight;
                // create the bitmap to display
                this.colorBitmap = new WriteableBitmap(this.colorWidth, this.colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);

                this.depthFrameDescription = this.myKinectv2.DepthFrameSource.FrameDescription;
                this.depthWidth = depthFrameDescription.Width;
                this.depthHeight = depthFrameDescription.Height;
                this.depthPixelCount = this.depthWidth * this.depthHeight;
                // allocate space to put the pixels being received and converted
                this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];


                // create the bitmap to display
                this.depthBitmap = new WriteableBitmap(this.depthWidth, this.depthHeight, 96.0, 96.0, PixelFormats.Gray8, null);

                this.infraredFrameDescription = this.myKinectv2.InfraredFrameSource.FrameDescription;
                this.infraredWidth = infraredFrameDescription.Width;
                this.infraredHeight = infraredFrameDescription.Height;
                this.infraredPixelCount = this.colorWidth * this.colorHeight;
                // create the bitmap to display
                this.infraredBitmap = new WriteableBitmap(this.infraredWidth, this.infraredHeight, 96.0, 96.0, PixelFormats.Gray32Float, null);

                // Add an event handler to be called whenever depth ,color or IR have new data
                this._reader.MultiSourceFrameArrived += this._reader_MultiSourceFrameArrived;

                // set IsAvailableChanged event notifier
                this.myKinectv2.IsAvailableChanged += this.Sensor_IsAvailableChanged;

                // use the window object as the view model in this simple example
                this.DataContext = this;

                this.depthVisibilityTestMapWidth = this.colorWidth / ColorDownsampleFactor;
                this.depthVisibilityTestMapHeight = this.colorHeight / ColorDownsampleFactor;
                this.depthVisibilityTestMap = new ushort[this.depthVisibilityTestMapWidth * this.depthVisibilityTestMapHeight];

                // set the status text
                //StatusText = this.myKinectv2.IsAvailable ? Properties.Resources.RunningStatusText
                  //                                              : Properties.Resources.NoSensorStatusText;

                // Setup the graphics rendering

                // Create virtualCamera for non-Kinect viewpoint rendering
                // Default position is translated along Z axis, looking back at the origin
                this.virtualCameraStartTranslation = new Point3D(0, 0, this.voxelsZ / this.voxelsPerMeter);
                this.virtualCamera = new GraphicsCamera(this.virtualCameraStartTranslation, this.virtualCameraStartRotation, (float)Width / (float)Height);

                // Attach this virtual camera to the viewport
                this.GraphicsViewport.Camera = this.virtualCamera.Camera;

                // Start worker thread for depth processing
                this.StartWorkerThread();

                // Start fps timer
                this.fpsTimer = new DispatcherTimer(DispatcherPriority.Send);
                this.fpsTimer.Interval = new TimeSpan(0, 0, FpsInterval);
                this.fpsTimer.Tick += this.FpsTimerTick;
                this.fpsTimer.Start();

                // Set last fps timestamp as now
                this.lastFPSTimestamp = DateTime.UtcNow;

                // Start status bar timer
                this.statusBarTimer = new DispatcherTimer(DispatcherPriority.Send);
                this.statusBarTimer.Interval = new TimeSpan(0, 0, StatusBarInterval);
                this.statusBarTimer.Tick += this.StatusBarTimerTick;
                this.statusBarTimer.Start();

                this.lastStatusTimestamp = DateTime.Now;

                // Allocate frames for Kinect Fusion now a sensor is present
                this.AllocateKinectFusionResources();

                // Create the camera frustum 3D graphics in WPF3D
                this.virtualCamera.CreateFrustum3DGraphics(this.GraphicsViewport, this.depthWidth, this.depthHeight);

                // Set recreate reconstruction flag
                this.recreateReconstruction = true;

                // Show introductory message
                //this.ShowStatusMessage(Properties.Resources.IntroductoryMessage);
                this.ShowStatusMessage("Click Reset Reconstruction to clear");


            }

            // Disable button to avoid errors
            btnConnect.IsEnabled = false;
            btnScreenshot.IsEnabled = true;
            btnSavePLY.IsEnabled = true;
            btnStart.IsEnabled = true;
        }

        /*------------------------------------------------------------------------------------------ */
        public void _reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            noOfFrames++;
            txtBox.Clear();
            txtBox.Append(noOfFrames);
            tb.Text = txtBox.ToString();

            bool validDepth = false;
            bool validColor = false;
            bool validInfrared = false;
			

			/*********************************************/
			PreProcessingFilters.PreProcessingFilters customFilters = new PreProcessingFilters.PreProcessingFilters();

			// check settigsn window for updates
			//checkSettingsWindow();

			// Get a reference to the multi-frame
			//var reference = e.FrameReference.AcquireFrame();
			MultiSourceFrame reference = e.FrameReference.AcquireFrame();

			try
            {
                if (reference != null)
                {
                    // MultiSourceFrame is IDisposable
                    lock (this.rawDataLock)
                    {

                        colorFrame = reference.ColorFrameReference.AcquireFrame();
                        FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                        int colorWidth = colorFrameDescription.Width;
                        int colorHeight = colorFrameDescription.Height;
                        if ((colorWidth * colorHeight * sizeof(int)) == this.colorImagePixels.Length)
                        {
                            colorFrame.CopyConvertedFrameDataToArray(this.colorImagePixels, ColorImageFormat.Bgra);
                            validColor = true;
                        }

                        depthFrame = reference.DepthFrameReference.AcquireFrame();
                        FrameDescription depthFrameDescription = depthFrame.FrameDescription;
                        int depthWidth = depthFrameDescription.Width;
                        int depthHeight = depthFrameDescription.Height;
                        if ((depthWidth * depthHeight) == this.depthImagePixels.Length)
                        {
                            depthFrame.CopyFrameDataToArray(this.depthImagePixels);
                            validDepth = true;
                            temp = depthImagePixels;
                        }
						
                        infraredFrame = reference.InfraredFrameReference.AcquireFrame();
                        FrameDescription infraredFrameDescription = infraredFrame.FrameDescription;
                        int infraredWidth = infraredFrameDescription.Width;
                        int infraredHeight = infraredFrameDescription.Height;
                        
                        if ((depthFrame != null) && (colorFrame != null) && (infraredFrame != null))
                        {
                            // ******************************* Display DEPTH images ********************************

                            //xamlFrame_DisplayDepth.Source = ToBitmap(depthFrame);

                            using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                            {
                                // verify data and write the color data to the display bitmap
                                if (((this.depthWidth * this.depthHeight) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                                    (this.depthWidth == this.depthBitmap.PixelWidth) && (this.depthHeight == this.depthBitmap.PixelHeight))
                                {
                                    // Note: In order to see the full range of depth (including the less reliable far field depth)
                                    // we are setting maxDepth to the extreme potential depth threshold
                                    ushort maxDepth = ushort.MaxValue;
                                    ushort minDepth = ushort.MinValue;

									// If you wish to filter by reliable depth distance, uncomment the following line:
									//maxDepth = depthFrame.DepthMaxReliableDistance;

									/*********************************************/
									// Setting custom clip depths <<
									if ( settingsWindow.ClipNoisyData == true)
									{
										maxDepth = 900;    // 1000 = 3.28ft
										minDepth = 450;     // 450 = 1.47ft

										// Clip the values above 1m (i.e. 1000 raw value); before passing on to Fusion 
										depthImagePixels = customFilters.clipAbove1m( depthImagePixels, maxDepth, minDepth );
										temp = depthImagePixels;	// Update temp
										//debugLog("Clippng data to 900mm depth");
									}
									else
									{
										maxDepth = 8000;
										minDepth = 500;
									}

									/*********************************************/
									// Applying Camera calibration parameters and undistorting the depth image acquired
									if ( settingsWindow.cameraCalibration == true )
									{
										try
										{
											depthImagePixels = customFilters.cameraCalibration1D( depthImagePixels );
										}
										catch ( Exception ex )
										{
											Trace.WriteLine( RawDepthHeight + "Unable to apply camera undistortions" + customFilters );
											debugLog( "Unable to apply camera undistortions" );
											debugLog( ex.Message );
											Console.WriteLine();
										}
									}
									else
									{
										depthImagePixels = temp;
									}

									/*********************************************/
									// Applying custom median filter to already clipped frame data
									// Median filter window height and width as parameters
									if ( settingsWindow.sortMedianFilter == true)
                                    {

                                        try
                                        {
                                            depthImagePixels = customFilters.MedianFilter_sort( depthImagePixels, 5, 5);
                                        }
                                        catch (Exception ex)
                                        {

                                            Trace.WriteLine(RawDepthHeight + "Sort Median Filter Error" + customFilters);
                                            debugLog("Could not apply median (sort) filter..");
											debugLog( ex.Message );
                                            Console.WriteLine();
                                        }
                                    }

									/*********************************************/
									if ( settingsWindow.fastmedianFilter == true)
                                    {
                                        try
                                        {
                                            depthImagePixels = customFilters.MedianFilter_fast( depthImagePixels, 5, 5);
                                        }
                                        catch (Exception ex)
                                        {
                                            Trace.WriteLine(RawDepthHeight + "Fast Median Filter error" + customFilters);
                                            debugLog("Could not apply median (fast) filter..");
											debugLog( ex.Message );
											Console.WriteLine();
                                        }
                                    }

									/*********************************************/
									if ( settingsWindow.refractionCorrection == true)
                                    {
                                        try
                                        {
                                            depthImagePixels = customFilters.RefractoionAndToFCorrection( depthImagePixels );
                                        }
                                        catch (Exception ex)
                                        {
                                            Trace.WriteLine(RawDepthHeight + "Fast Median Filter error" + customFilters);
                                            debugLog("Could not apply Refraction Correction..");
											debugLog( ex.Message );
											Console.WriteLine();
                                        }
                                    }

									/*********************************************/

									//this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
									this.ProcessDepthFrameData(depthImagePixels, depthBuffer.Size, minDepth, maxDepth);

                                    this.RenderDepthPixels();
                                }
                            }

                            // ******************************* Display Colour images ********************************

                            using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                            {
                                this.colorBitmap.Lock();

                                //verify data and write the new color frame data to the display bitmap
                                if ((colorWidth == this.colorBitmap.PixelWidth) && (colorHeight == this.colorBitmap.PixelHeight))
                                {
                                    colorFrame.CopyConvertedFrameDataToIntPtr(
                                        this.colorBitmap.BackBuffer,
                                        (uint)(colorWidth * colorHeight * 4),
                                        ColorImageFormat.Bgra);

                                    this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                                }

                                this.colorBitmap.Unlock();
                            }

                            // ******************************* Display Infrared images ********************************

                            using (Microsoft.Kinect.KinectBuffer infraredBuffer = infraredFrame.LockImageBuffer())
                            {
                                // verify data and write the new infrared frame data to the display bitmap
                                if (((this.infraredWidth * this.infraredHeight) == (infraredBuffer.Size / this.infraredFrameDescription.BytesPerPixel)) &&
                                    (this.infraredWidth == this.infraredBitmap.PixelWidth) && (this.infraredHeight == this.infraredBitmap.PixelHeight))
                                {
                                    this.ProcessInfraredFrameData(infraredBuffer.UnderlyingBuffer, infraredBuffer.Size);
                                }
                            }

                            // ******************************* Display -------- images ********************************
                            if (colour_Image.IsSelected)
                            {
                                xamlFrame_DisplayColor.Source = colorBitmap;
                            }
                            else if (IR_Image.IsSelected)
                            {
                                xamlFrame_DisplayInfrared.Source = infraredBitmap;
                            }
							else if ( deltaFrameRef.IsSelected )
							{
								xamlFrame_deltaFromReferenceImage.Source = deltaFromReferenceFrameBitmap;
							}

                            // *********************************************************************************

                            // Save frame timestamp
                            this.RelativeTime = depthFrame.RelativeTime;
                                                        
                        }
                    }
                }
            }
            catch (Exception)
            {
                // ignore if the frame is no longer available
            }
            finally
            {
                // MultiSourceFrame, DepthFrame, ColorFrame, BodyIndexFrame are IDispoable
                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                    depthFrame = null;
                }

                if (colorFrame != null)
                {
                    colorFrame.Dispose();
                    colorFrame = null;
                }

                if (infraredFrame != null)
                {
                    infraredFrame.Dispose();
                    infraredFrame = null;
                }

                if (reference != null)
                {
                    reference = null;
                }
            }

            if (validDepth)
            {
                // Signal worker thread to process
                this.depthReadyEvent.Set();
            }

            if (validColor)
            {
                // Signal worker thread to process
                this.colorReadyEvent.Set();
            }

            if (validInfrared)
            {
                // Signal worker thread to process
                this.infraredReadyEvent.Set();
            }

        }

        /*------------------------------------------------------------------------------------------ */

        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(ushort[] depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {

            /* Raw Depth Matrix?? */


            // depth frame data is a 16 bit value
            //ushort* frameData = (ushort*)depthFrameData;
            ushort[] frameData = depthFrameData;
            /******************************************************/
            /// Preprocess Depth image acquired by Media filter IF checkbox is checked
            /// true to be replaced by checkbox variable when GUI is updated. 
            /// Currently it will be on always
            if (true)
            {
               // PreProcessingFilters.PreProcessingFilters.MedianFilter(frameData, 3);
            }
            /******************************************************/

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }

        /*------------------------------------------------------------------------------------------ */

        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
        }

        /*------------------------------------------------------------------------------------------ */
        /// Directly accesses the underlying image buffer of the InfraredFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the infraredFrameData pointer.
        /// </summary>
        /// <param name="infraredFrameData">Pointer to the InfraredFrame image data</param>
        /// <param name="infraredFrameDataSize">Size of the InfraredFrame image data</param>
        private unsafe void ProcessInfraredFrameData(IntPtr infraredFrameData, uint infraredFrameDataSize)
        {
            // infrared frame data is a 16 bit value
            ushort* frameData = (ushort*)infraredFrameData;

            // lock the target bitmap
            this.infraredBitmap.Lock();

            // get the pointer to the bitmap's back buffer
            float* backBuffer = (float*)this.infraredBitmap.BackBuffer;

            // process the infrared data
            for (int i = 0; i < (int)(infraredFrameDataSize / this.infraredFrameDescription.BytesPerPixel); ++i)
            {
                // since we are displaying the image as a normalized grey scale image, we need to convert from
                // the ushort data (as provided by the InfraredFrame) to a value from [InfraredOutputValueMinimum, InfraredOutputValueMaximum]
                backBuffer[i] = Math.Min(InfraredOutputValueMaximum, (((float)frameData[i] / InfraredSourceValueMaximum * InfraredSourceScale) * (1.0f - InfraredOutputValueMinimum)) + InfraredOutputValueMinimum);
            }

            // mark the entire bitmap as needing to be drawn
            this.infraredBitmap.AddDirtyRect(new Int32Rect(0, 0, this.infraredBitmap.PixelWidth, this.infraredBitmap.PixelHeight));

            // unlock the bitmap
            this.infraredBitmap.Unlock();
        }

        /*------------------------------------------------------------------------------------------ */

        private void btnSettings_Click(object sender, RoutedEventArgs e)
        {
            // Toggle variable to show window and then show window. 
            // Passing Mainwindow parameter to sub window
            
            if (showSettingsWindow == true)
            {
                settingsWindow.Visibility = Visibility.Collapsed;
                showSettingsWindow = false;
                statusBarText.Text = "Settings window is visible; closing";

				// positioning window where the main window is
				settingsWindow.Top = this.Top + 150;
				settingsWindow.Left = this.Left - 190;
                debugLog(" Settings window is visible; closing.");
            }
            else
            {
                settingsWindow.Visibility = Visibility.Visible;
                showSettingsWindow = true;
                statusBarText.Text = "Settings window is NOT visible; launching";

				// positioning window where the main window is
				settingsWindow.Top = this.Top + 150;
				settingsWindow.Left = this.Left - 190;
				// display on logging window
				debugLog(" Settings window is NOT visible; launching.");
            }
        }

        /*------------------------------------------------------------------------------------------ */

        private void btnStatsShow_Click(object sender, RoutedEventArgs e)
        {
            
        }

        /*------------------------------------------------------------------------------------------ */

       

        /*------------------------------------------------------------------------------------------ */


        private void btnConsoleShow_Click(object sender, RoutedEventArgs e)
        {
            // Toggle variable to show window and then show window.
            statsWindow.Owner = this;

            if (showStatsWindow == true)
            {
                statsWindow.Visibility = Visibility.Collapsed;
                showStatsWindow = false;
                statusBarText.Text = "Stats window is visible; closing now";

				// positioning window where the main window is
				statsWindow.Top = this.Top + 150;
				statsWindow.Left = this.Left + 1190;
				debugLog("Stats window is visible; closing now");

            }
            else
            {
                statsWindow.Visibility = Visibility.Visible;
                showStatsWindow = true;
                statusBarText.Text = "Stats window is NOT visible; launching now";

				// positioning window where the main window is
				statsWindow.Top = this.Top + 150;
				statsWindow.Left = this.Left + 1190;
				// display on logging window
				debugLog("\n>> Stats window is NOT visible; launching now.");
            }

        }

        

        /*------------------------------------------------------------------------------------------ */

        private void btnStart_Click(object sender, RoutedEventArgs e)
        {
            // Call the Kinect Fusion process
            //Process();
            if (!this.PauseIntegration)
            {
                this.btnStart.FontSize = 16;
                this.PauseIntegration = true;
                this.btnStart.Content = "Continue \nReconstruction";
                return;
            }
            else
            {
                this.btnStart.FontSize = 16;
                this.PauseIntegration = false;
                this.btnStart.Content = "Pause \nReconstruction";
                return;
            }
            

        }

        /*------------------------------------------------------------------------------------------ */

        private void btnStop_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
            Application.Current.Shutdown();
        }

        /*------------------------------------------------------------------------------------------ */

        private void btnSavePLY_Click(object sender, RoutedEventArgs e)
        {
            // Mark the start time of saving mesh
            DateTime beginning = DateTime.UtcNow;

            try
            {
                this.ShowStatusMessage("Creating and saving mesh of reconstruction, please wait...");
				debugLog( "Creating and saving mesh of reconstruction, please wait..." );

                ColorMesh mesh = null;

                lock (this.volumeLock)
                {
                    this.savingMesh = true;

                    if (null == this.volume)
                    {
                        this.ShowStatusMessage("Cannot create mesh of non-existent reconstruction");
						debugLog( "Cannot create mesh of non-existent reconstruction" );
                        return;
                    }

                    mesh = this.volume.CalculateMesh(1);
                }

                if (null == mesh)
                {
                    this.ShowStatusMessage("Error saving Kinect Fusion mesh!");
					debugLog( "Error saving Kinect Fusion mesh!" );
                    return;
                }

                Microsoft.Win32.SaveFileDialog dialog = new Microsoft.Win32.SaveFileDialog();

                // **************** STL format filesave (aka 3D MESH) **************** 
                
                dialog.FileName = "_KinectIsAwesome.stl";
                dialog.Filter = "STL Mesh Files|*.stl";
                if (true == dialog.ShowDialog())
                {
                    using (BinaryWriter writer = new BinaryWriter(dialog.OpenFile()))
                    {
                        // Default to flip Y,Z coordinates on save
                        KinectFusionHelper.SaveBinaryStlMesh(mesh, writer, true);
                    }
                    this.ShowStatusMessage("Saved STL file");
                    //debugLog("Saved STL file");
                }
                else
                {
                    this.ShowStatusMessage("STL Mesh save canceled");
                    //debugLog("STL Mesh save canceled");
                }

                // **************** OBJ format filesave (for Point Cloud Viewer)**************** 
                
                dialog.FileName = "_KinectIsAwesome.obj";
                dialog.Filter = "OBJ Mesh Files|*.obj";
                
                if (true == dialog.ShowDialog())
                {
                    using (StreamWriter writer = new StreamWriter(dialog.FileName))
                    {
                        // Default to flip Y,Z coordinates on save
                        KinectFusionHelper.SaveAsciiObjMesh(mesh, writer, true);
                    }
                    this.ShowStatusMessage("Saved OBJ File");
                }
                else
                {
                    this.ShowStatusMessage("OBJ file save canceled");
                    debugLog("OBJ file save canceled");
                }

                // **************** PLY format filesave (vertices in txt format)**************** 
                
                dialog.FileName = "_KinectIsAwesome.ply";
                dialog.Filter = "PLY Mesh Files|*.ply";
                
                if (true == dialog.ShowDialog())
                {
                    using (StreamWriter writer = new StreamWriter(dialog.FileName))
                    {
                        // Default to flip Y,Z coordinates on save
                        KinectFusionHelper.SaveAsciiPlyMesh(mesh, writer, true, this.colorCaptured);
                    }
                    this.ShowStatusMessage("Saved PLY File");
                }
                else
                {
                    this.ShowStatusMessage("PLY file save canceled");
                    debugLog("PLY file save canceled");
                }
            }
            // End try();

            catch (ArgumentException)
            {
                this.ShowStatusMessage("Error saving Kinect Fusion mesh!");
            }
            catch (InvalidOperationException)
            {
                this.ShowStatusMessage("Error saving Kinect Fusion mesh!");
            }
            catch (IOException)
            {
                this.ShowStatusMessage("Error saving Kinect Fusion mesh!");
            }
            catch (OutOfMemoryException)
            {
                this.ShowStatusMessage("Error saving Kinect Fusion mesh - out of memory!");
            }
            finally
            {
                // Update timestamp of last frame to avoid auto reset reconstruction
                this.lastFrameTimestamp += DateTime.UtcNow - beginning;

                this.savingMesh = false;
            }
        }
    
        /*------------------------------------------------------------------------------------------ */
        // <summary>
        // Gets or sets the current status text to display
        // </summary>
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
					//debugLog( value );

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /*------------------------------------------------------------------------------------------ */
        // Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // set the status text
            //StatusText = this.myKinectv2.IsAvailable ? Properties.Resources.RunningStatusText
                                                            //: Properties.Resources.SensorNotAvailableStatusText;
        }

        /*------------------------------------------------------------------------------------------ */

        /// Gets or sets a value indicating whether to display surface normals.
        public bool DisplayNormals
        {
            get
            {
                return this.displayNormals;
            }

            set
            {
                this.displayNormals = value;
                if (null != this.PropertyChanged)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("DisplayNormals"));
                }
            }
        }
        /*------------------------------------------------------------------------------------------ */

        /// Gets or sets a value indicating whether to capture color.
        public bool CaptureColor
        {
            get
            {
                return this.captureColor;
            }

            set
            {
                this.captureColor = value;

                if (null != this.PropertyChanged)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("CaptureColor"));
                }
            }
        }

        /*------------------------------------------------------------------------------------------ */

        /// Gets or sets a value indicating whether to pause integration.
        public bool PauseIntegration
        {
            get
            {
                return this.pauseIntegration;
            }

            set
            {
                this.pauseIntegration = value;
                if (null != this.PropertyChanged)
                {
                    //this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("PauseIntegration"));
                }
            }
        }

        /*------------------------------------------------------------------------------------------ */

        /// Gets or sets a value indicating whether to mirror depth.
        public bool MirrorDepth
        {
            get
            {
                return this.mirrorDepth;
            }

            set
            {
				this.mirrorDepth = value;
                if (null != this.PropertyChanged)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("MirrorDepth"));
                }

				this.resetReconstruction = true;
            }
        }

        /*------------------------------------------------------------------------------------------ */
        private void btnScreenshot_Click(object sender, RoutedEventArgs e)
        {
			
            //string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            //string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

            if (this.colorBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.colorBitmap));

                string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = Path.Combine(myPhotos, "KinectScreenshot-color-" + time + ".png");

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                    this.StatusText = string.Format("Screenshot Saved in My Pictures folder");
					debugLog( "Screenshot Saved in My Pictures folder" );
				}
                catch (IOException)
                {
                    this.StatusText = string.Format("Failed to write screenshot");
					debugLog( "Failed to write screenshot" );
				}
            }

            if (this.depthBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.depthBitmap));

                string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = Path.Combine(myPhotos, "KinectScreenshot-Depth-" + time + ".png");

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                    this.StatusText = string.Format("Screenshot Saved in My Pictures folder");
					debugLog( "Screenshot Saved in My Pictures folder" );
				}
                catch (IOException)
                {
                    this.StatusText = string.Format("Failed to write screenshot");
					debugLog( "Failed to write screenshot" );
				}
            }

            if (this.infraredBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.infraredBitmap));

                string path = Path.Combine(myPhotos, "KinectScreenshot-Infrared-" + time + ".png");

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                    this.StatusText = string.Format("Screenshot Saved in My Pictures folder");
					debugLog( "Screenshot Saved in My Pictures folder" );
                }
                catch (IOException)
                {
                    this.StatusText = string.Format("Failed to write screenshot");
					debugLog( "Failed to write screenshot" );
                }
            }

        }


        /*------------------------------------------------------------------------------------------ */
        // to Pass text to stats for logging text
        public void debugLog (string debugText)
        {
            //statsWindow.debugLog.Text += debugText;
            //statsWindow.debugLog.Text += "\n: ";
            //statsWindow.scrollViewer.ScrollToEnd();

			statsWindow.scrollViewer.Content += debugText;
			statsWindow.scrollViewer.Content += "\n: ";
			statsWindow.scrollViewer.ScrollToEnd();

			// add file log text here 

		}

        /*------------------------------------------------------------------------------------------ */
        // Function to check the settings window for any settings update
        void checkSettingsWindow ()
        {
            if (settingsWindow.somethinChanged == true)
            {
                // Get updated value from Settings Window
                //this.voxelsX = settingsWindow.voxelsX;
                debugLog("Settings voxelX");

                // Get updated value from Settings Window
                //this.voxelsY = settingsWindow.voxelsY;
                debugLog("Settings voxelX");

                // Get updated value from Settings Window
                //this.voxelsZ = settingsWindow.voxelsZ;
                debugLog("Settings voxelX");

                //this.voxelsPerMeter = this.settingsWindow.voxelsPerMeter;
                debugLog("Setting Voxel per meter");

                //this.integrationWeight = settingsWindow.integrationWeight;
                debugLog("Setting Integration Weight");

                //this.CaptureColor = settingsWindow.CaptureColor;
                debugLog("Starting Color Integration");

                //this.resetReconstruction = settingsWindow.resetReconstruction;
                debugLog("Resetting Integration");
                // resetting flag
                //this.settingsWindow.resetReconstruction = false;

                // reset the flag
                settingsWindow.somethinChanged = false;
            }
        }


        /*------------------------------------------------------------------------------------------ */
    }
}
