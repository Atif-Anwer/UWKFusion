using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Diagnostics;

namespace UWKFusion
{
    /// <summary>
    /// Interaction logic for StatsWindow.xaml
    /// </summary>
    /// 
    
    public partial class SettingsWindow : Window
    {

        // Height and width of screen
        public double _screenHeight;
        public double _screenWidth;
        public double _screenCenterX;
        public double _screenCenterY;
		public double a;
        //private object _mainWindow;

        ///// The reconstruction volume voxel resolution in the X axis
        ///// At a setting of 256vpm the volume is 384 / 256 = 1.5m wide
        //public int voxelsX = 384;

        ///// The reconstruction volume voxel resolution in the Y axis
        ///// At a setting of 256vpm the volume is 384 / 256 = 1.5m high
        //public int voxelsY = 384;

        ///// The reconstruction volume voxel resolution in the Z axis
        ///// At a setting of 256vpm the volume is 384 / 256 = 1.5m deep
        //public int voxelsZ = 512;

        ///// The reconstruction volume voxel density in voxels per meter (vpm)
        ///// 1000mm / 256vpm = ~3.9mm/voxel
        //public int voxelsPerMeter = 256;

        public short integrationWeight = 10;
        public bool CaptureColor = false;
        public bool somethinChanged = false;
        public bool mirrorDepth = true;
        public bool kinectView;
        public bool sortMedianFilter;
        public bool fastmedianFilter;
        public bool resetReconstruction;
        public bool refractionCorrection = false;
		public bool ClipNoisyData = false;
		public bool volumeGraphics = false;
		public bool UseCameraPoseFinder;
		public bool cameraCalibration;

		//MainWindow mw = new MainWindow();
		//static MainWindow _instance = new MainWindow();

		public SettingsWindow()
        {
			InitializeComponent();

			// Get screen width, height and center to position the sceens.
			//this.Top = this.Owner.Top + 20;

			//_screenHeight = SystemParameters.WorkArea.Height;
   //         _screenWidth = SystemParameters.WorkArea.Width;
   //         _screenCenterX = _screenWidth / 2;
   //         _screenCenterY = _screenHeight / 2;

            // Settings WIndow Height="600" Width="160"
            // Main WIndow Height="720" Width="1280"
            //Top = _screenCenterY - (600 / 3);
            //Left = _screenCenterX - (1200 / 2) - 190;

			//a = _instance.Top;
			//this.Top = _instance.Top;
			//Left = _instance.Top - 5; 
			//VoxelsXSlider.Value = 3;

			//voxelsX = 384;
			//voxelsY = 384;
			//voxelsZ = 512;

			//this.VoxelsXSlider.Value = voxelsX;
			//this.VoxelsYSlider.Value = voxelsY;
			//this.VoxelsZSlider.Value = voxelsZ;

			//this.VoxelsPerMeterSlider.Value = 256;
			// set checkbox value to true by default
			//chkboxMirrorDepth.SetCurrentValue( CheckBox.IsCheckedProperty, true );
			//chkboxKinectView.SetCurrentValue( CheckBox.IsCheckedProperty, true );

		}

		/****************** VOXELS ****************************/
  //      private void VoxelsXSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
  //      {
  //          this.voxelsX = (int)this.VoxelsXSlider.Value;
  //          somethinChanged = true;
  //      }

  //      private void VoxelsYSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
  //      {
  //          this.voxelsY = (int)this.VoxelsYSlider.Value;
  //          somethinChanged = true;
  //      }

  //      private void VoxelsZSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
  //      {
  //          this.voxelsZ = (int)this.VoxelsZSlider.Value;
  //          somethinChanged = true;
  //      }

  //      private void VolumeIntegrationWeight_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
  //      {
  //          this.integrationWeight = (short)this.VolumeIntegrationWeightSlider.Value;
  //          somethinChanged = true;
		//}

  //      private void VoxelPerMeter_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
  //      {
  //          this.voxelsPerMeter = (int)this.VoxelsZSlider.Value;
  //          somethinChanged = true;
  //      }

		/****************** BUTTONS ****************************/

		private void btnResetReconstruction_Click(object sender, RoutedEventArgs e)
        {
			((MainWindow)Owner).ResetReconstructionButtonClick( sender, e );
        }

        private void btnResetVirtualCamera_Click(object sender, RoutedEventArgs e)
        {
			//((MainWindow)Owner).ResetCameraButtonClick( sender, e );
			((MainWindow)Owner).virtualCamera.Reset();
		}

		/****************** CUSTOM FILTERS ****************************/
		private void SortMedianFilter_Checked(object sender, RoutedEventArgs e)
        {
            if (chkboxSortMedianFilter.IsChecked == true)
            {
                sortMedianFilter = true;
                somethinChanged = true;
				((MainWindow)Owner).ResetReconstructionButtonClick( sender, e );
			}
            else
            {
                sortMedianFilter = false;
                somethinChanged = true;
				((MainWindow)Owner).ResetReconstructionButtonClick( sender, e );
			}
        }

        private void FastMedianFilter_Checked(object sender, RoutedEventArgs e)
        {
            if (chkboxFastMedianFilter.IsChecked == true)
            {
                fastmedianFilter= true;
                somethinChanged = true;
            }
            else
            {
                fastmedianFilter = false;
                somethinChanged = true;
            }
        }

        private void RefractionCorrection_Checked(object sender, RoutedEventArgs e)
        {
            if (chkboxRefractionCorrection.IsChecked == true)
            {
                refractionCorrection = true;
                somethinChanged = true;
				((MainWindow)this.Owner).ResetReconstructionButtonClick( sender, e );
			}
            else
            {
                refractionCorrection = false;
                somethinChanged = true;
				((MainWindow)this.Owner).ResetReconstructionButtonClick( sender, e );
			}

        }

		private void ClipNoisyData_Checked(object sender, RoutedEventArgs e)
		{
			if (chkboxClipData.IsChecked == true)
            {
				ClipNoisyData = true;
                somethinChanged = true;
            }
            else
            {
				ClipNoisyData = false;
                somethinChanged = true;
            }
		}

		private void chkboxCameraCalibration_Checked( object sender, RoutedEventArgs e )
		{
			if ( chkboxCameraCalibration.IsChecked == true )
			{
				cameraCalibration = true;
				somethinChanged = true;
				((MainWindow)this.Owner).ResetReconstructionButtonClick( sender, e );
			}
			else
			{
				cameraCalibration = false;
				somethinChanged = true;
				((MainWindow)this.Owner).ResetReconstructionButtonClick( sender, e );
			}
		}
	}
}

