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

namespace UWKFusion
{
    /// <summary>
    /// Interaction logic for StatsWindow.xaml
    /// </summary>
    public partial class StatsWindow : Window
    {
        // Height and width of screen
        public double _screenHeight;
        public double _screenWidth;
        public double _screenCenterX;
        public double _screenCenterY;

        public StatsWindow()
        {
            this.InitializeComponent();

            // Get screen width, height and center to position the sceens.
            _screenHeight = System.Windows.SystemParameters.WorkArea.Height;
            _screenWidth = System.Windows.SystemParameters.WorkArea.Width;
            _screenCenterX = _screenWidth / 2;
            _screenCenterY = _screenHeight / 2;

            // Settings WIndow Height="600" Width="160"
            // Main WIndow Height="720" Width="1280"
            Top = _screenCenterY - (600 / 3);
            Left = _screenCenterX + (1180 / 2);

        }

        private void ScrollViewer_RequestBringIntoView(object sender, RequestBringIntoViewEventArgs e)
        {

        }

		private void saveDebugLog_Click( object sender, RoutedEventArgs e )
		{

		}
	}
}
