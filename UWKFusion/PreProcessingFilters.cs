/*------------------------------------------------------------------------------
*    Class for pre-procesing the point cloud acquired. Contains the following functions
	 1. Median Filter for clearing salt and pepper noise
	 2. Refraction Correction
	 3. Time of flight depth adjustment
*    Author: Atif Anwer
*    Version: 2.0
*    Project: Underwater Kinect Fusion
*    Masters Thesis
*    Git Source @ BitBucket
------------------------------------------------------------------------------*/


using Kinect.KinectFusionExplorer;
using Microsoft.Kinect;
using Microsoft.Kinect.Fusion;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Diagnostics;
using HelvioJunior;

namespace UWKFusion.PreProcessingFilters
{
	public unsafe class PreProcessingFilters
	{
		/// Width of raw depth stream
		const int RawDepthWidth = 512;

		/// Height of raw depth stream
		const int RawDepthHeight = 424;

		// Refraction indices of the three media
		const double nIndexAir = 1.0003;                                        // refraction index of air
		const double nIndexPerspex = 1.492;                                     // refraction index of perspex
		const double nIndexWater = 1.333;                                       // refraction index of water

		// Field of view
		const double HFOV = 1.2322;                                             // Horizntal Kinect FOV deg2rad(70.6)
		const double VFOV = 1.0472;                                             // Vertical FOV Kinect deg2rad(60)
		const double HFOVcenter = HFOV / 2;                                     // 35.3 degrees
		const double VFOVcenter = VFOV / 2;                                     // 30.0 degrees
		const double horizongtalAngleResolution = HFOV / RawDepthWidth;         // 0.138 deg/px | 0.0024 rad/px
		const double verticalAngleResolution = VFOV / RawDepthHeight;           // 0.142 deg/px | 0.0025 rad/px

		// distance of the designed casing. Perspex is 2mm thick.
		const double dist_air = 0.05;                                           // Inside distance from Kinect to Casing in mm
		const double dist_perspex = 2.0;                                        // thickness of perspex housing in mm
		const double measuredDistance = 0.0;                                    // Original distance measured by Kinect

		// flags
		const int perspex = 1;
		const int water = 2;

		// speed of light in air
		const double c_air = 2.997e11;                                          // mm/s^2
		const double c_water = (nIndexAir / nIndexWater) * (2.99e11);
		const double c_perspex = (nIndexAir / nIndexPerspex) * (2.99e11);

		ushort[] outputImage = new ushort[RawDepthHeight * RawDepthWidth];

		/// Median filter window size
		ushort med;
		int delta_l;
		int[] hist = new int[8001];
		int row = 0;
		int col = 0;
		int directionValue = 1;
		int linenum = 0;

		//byte[RawDepthHeight][RawDepthWidth] m_data = 0;
		//byte[][] m_data = new byte[RawDepthHeight][];
		//byte[][] m_data = null;
		//ushort[][] tmp2 = null;
		public ushort[][] m_data = new ushort[RawDepthHeight][];
		public ushort[][] tmpMatrix = new ushort[RawDepthHeight][];

		// initalizing and pre-alocating size
		public double[][] thetaAir = new double[RawDepthHeight][];
		public double[][] phiAir = new double[RawDepthHeight][];
		public double[][] rzx_air = new double[RawDepthHeight][];
		public double[][] rzy_air = new double[RawDepthHeight][];

		private double[][] depthImagePixels2D;
		private ushort[] depthImagePixels;

		///NOTE:    ushort is a 2 byte (16-bit) variable that can hold whole values between 0 and 65535 (unsigned)
		///         double is a 8-byte (64-bit) variable that can hold ±5.0 × 10e−324 to ±1.7 × 10e308, including fractions
		///         float is a 4-byte (32-bit) variable that can hold -3.4 × 10e38to +3.4 × 10e38 keyword signifies a simple type that stores 32-bit floating-point values
		///         We can use floats instead of double to save memory (?) but Math library returns double (sin and cos). So typecasting will probably slow the performance.
		///         * can look into this if performance or memmory issues *



		/**********************clipAbove1m***************************/
		/// <summary>
		///		* Clip the values above 1m (i.e. 1000 raw value); before passing on to Fusion
		/// </summary>
		/// <param name="depthImagePixels"> Original Image</param>
		/// <param name="maxDepth">Maximum depth threshold cutoff</param>
		/// <param name="minDepth"> Minimum depth threshold cutoff</param>
		/// <returns> the clipped depth matrix</returns>
		public ushort[] clipAbove1m(ushort[] depthImagePixels, ushort maxDepth, ushort minDepth)
		{
			for (int i = 0; i < (depthImagePixels.Length); i++)
			{
				if (depthImagePixels[i] != 0 && (depthImagePixels[i] < minDepth || depthImagePixels[i] > maxDepth) )
				{
					depthImagePixels[i] = 0;
				}
			}
			return depthImagePixels;
		}

		/**********************MedianFilter_sort***********************/
		/// <summary>
		///  MedianFilter3 FUNCTION
		///  Classic Median filter by sorting 3x3 neighbouthood and getting median
		///  Inspired from: https://softwarebydefault.com/2013/05/18/image-median-filter/
		/// </summary>
		/// <param name="inputImage"> The input Image</param>
		/// <param name="filter_width"> Median Fiter width</param>
		/// <param name="filter_height">Median Filter heignt</param>
		/// <returns> Depth image ushort array </returns>
		public ushort[] MedianFilter_sort( ushort[] inputImage, int filter_width, int filter_height )
		{

			int wx2 = filter_width / 2;     // for 3x3; wx2 = 1
			int wy2 = filter_height / 2;    // for 3x3; wy2 = 1
			int middle = (filter_height * filter_width + 1) / 2;

			int index = 0;

			for ( int i = 0 ; i < m_data.Length ; i++ )
			{
				m_data[i] = new ushort[512];
				Array.Copy( inputImage, i * 512, m_data[i], 0, 512 );
			}

			// tmpMatrix for revurning modified value
			tmpMatrix = m_data;

			List<ushort> neighbourhood = new List<ushort>();
			wx2 = filter_width / 2;
			wy2 = filter_height / 2;

			try
			{
				for ( int row = 0 ; row < RawDepthHeight ; row++ )
				{
					for ( int col = 0 ; col < RawDepthWidth ; col++ )
					{
						for ( int r = row - wy2 ; r <= row + wy2 ; r++ )
							for ( int c = col - wx2 ; c <= col + wx2 ; c++ )
								neighbourhood.Add( getValue( r, c ) );
						neighbourhood.Sort();
						setValue( row, col, neighbourhood[(filter_width * filter_height) / 2] );
						neighbourhood.Clear();
					}
				}
			}
			catch ( Exception )
			{
				Trace.WriteLine( RawDepthHeight + "," + RawDepthWidth + "," + row + "," + col );
				//Console.WriteLine();

			}


			try
			{
				index = 0;
				for ( row = 0 ; row < RawDepthHeight ; row++ )
				{
					for ( col = 0 ; col < RawDepthWidth ; col++ )
					{
						//outputImage[i] = new ushort[RawDepthHeight*RawDepthWidth];
						//Array.Copy(m_data[i][j], i * 424, outputImage[index], 0, 424);
						outputImage[index] = tmpMatrix[row][col];
						index++;
					}
				}
			}
			catch ( Exception )
			{

				Trace.WriteLine( index + "," + row + "," + col + "," );
				//Console.WriteLine();
			}


			return outputImage;
		}

		/**********************MedianFilter_fast***********************/
		// <summary>
		//      * :: Huang Fast Median Filter Implementation using Histogram::
		//      * In 1981 T.Huang proposed fast 2D median filtering algorithm
		//      * for gray scale images.The main idea of algorithm is to use
		//      * histogram instead using sort. For example, we have array of
		//      * N elements taking values from 0 to 255. We put all elements
		//      * of array to histogram.Now, sum all histogram elements beginning
		//      * from 0 till sum reaches the middle N/2. This value will give us
		//      * median of array.
		//      * http://www.sergejusz.com/engineering_tips/median_filter.htm
		// </summary>
		/// <param name="inputImage"></param>
		/// <param name="filter_width"></param>
		/// <param name="filter_height"></param>
		/// <returns></returns>
		public ushort[] MedianFilter_fast(ushort[] inputImage, int filter_width, int filter_height)
		{

			/// The input image is a byte 1-D array (ushort) .
			/// To access in Row Col fashion, we need to convert the byte array to
			/// 2-D array and back when returning data.


			int index = 0;
			// index to parse inputimage
			try
			{
				for (int row = 0; row < m_data.Length; row++)
				{
					m_data[row] = new ushort[512];
					Array.Copy(inputImage, row * 512, m_data[row], 0, 512);
				}
			}
			catch (Exception)
			{

				throw;
			}

			// create a duplicate image matrix for returning later
			tmpMatrix = m_data;

			//for (int row = 0; row < RawDepthHeight; row++)
			//{
			//    for (int col = 0; col < RawDepthWidth; col++)
			//    {
			//        m_data[row] = inputImage[index];
			//        index++;
			//    }
			//}

			int wx2 = filter_width / 2;     // for 3x3; wx2 = 1

			int wy2 = filter_height / 2;    // for 3x3; wy2 = 1
			int middle = (filter_height * filter_width + 1) / 2;

			/// As the median filter is being called after clipping depth data
			/// therefore, the values that it will get will be betwwn
			/// maxDepth and minDepth values of ProcessDepthFrameData()
			/// function. So histogram values will be between:
			/// maxDepth in Raw Value =  8000   // 8000 = 24ft
			/// maxDepth = 1000;    // 1000 = 3.28ft
			/// minDepth = 450;     // 450 = 1.47ft
			/// histogram bins will be 1000; with no values inbetween 0 and 450 (?)
			for (int j = 0; j < 1000; j++)
				hist[j] = 0;

			// Histogram For (0,0)-element
			for (row = -wy2; row <= wy2; row++)
				for (col = -wx2; col <= wx2; col++)
					hist[getValue(row, col)]++;

			// Median
			int m = 0;
			for (med = 0; med <= 1000; med++)
			{
				m += hist[med];
				if (m >= middle)
					break;
			}
			delta_l = m - hist[med];

			// Now, Median Is Defined For (0,0)-element
			// Begin Scanning: direction - FORWARD
			setValue(0, 0, med);

			int prev, next;

			// main loop
			col = 1;
			for (row = 0; row < RawDepthHeight; row++)
			{
				while (col >= 0 && col < RawDepthWidth)
				{
					// Determine Previous and Next Columns
					// Pay Attention To Defined Direction !!!
					prev = col - directionValue * (wx2 + 1);
					next = col + directionValue * wx2;
					// Now Change Old Histogram
					// New Histogram
					// delete previous
					for (int r = row - wy2; r <= row + wy2; r++)
					{
						ushort value_out = getValue(r, prev);
						ushort value_in = getValue(r, next);
						if (value_out == value_in)
							continue;
						hist[value_out]--;
						if (value_out < med)
							delta_l--;
						hist[value_in]++;
						if (value_in < med)
							delta_l++;
					}

					// Update new median
					if (delta_l >= middle)
					{
						while (delta_l >= middle)
						{
							if (hist[--med] > 0)
								delta_l -= hist[med];
						}
					}
					else
					{
						while (delta_l + hist[med] < middle)
						{
							if (hist[med] > 0)
								delta_l += hist[med];
							med++;
						}
					}
					setValue(row, col, med);
					// end of column loop
					col += directionValue;
				}

				if (row < RawDepthHeight - 1)
				{
					// go back to the last/first pixel of the line
					col -= directionValue;
					// change direction to the opposite
					directionValue *= -1;

					// Shift Down One Line
					prev = row - wy2;
					next = row + wy2 + 1;


					for (int c = col - wx2; c <= col + wx2; c++)
					{
						ushort value_out = getValue(prev, c);
						ushort value_in = getValue(next, c);
						if (value_out == value_in)
							continue;
						hist[value_out]--;
						if (value_out < med)
							delta_l--;
						hist[value_in]++;
						if (value_in < med)
							delta_l++;
					}

					try
					{
						if (delta_l >= middle)
						{
							while (delta_l >= middle)
							{
								if (hist[--med] > 0)
									delta_l -= hist[med];
							}
						}
						else
						{
							while (delta_l + hist[med] < middle)
							{
								if (hist[med] > 0)
									delta_l += hist[med];
								med++;
							}
						}
						setValue(row + 1, col, med);
						col += directionValue;
					}
					catch (Exception)
					{
						Console.Write(RawDepthHeight + "," + RawDepthWidth + "," + row + "," + col);
						Console.WriteLine();
						throw;
					}
				}
			}

			/// Converting back to 2-D array when returning data.
			// reset index to parse inputimage
			index = 0;

			try
			{
				for (int i = 0; i < RawDepthHeight; ++i)
				{
					for (int j = 0; j < RawDepthWidth; j++)
					{
						outputImage[index] = (ushort)m_data[i][j];
						index++;
					}
				}
			}
			catch (Exception e)
			{
				var lineNumber = new System.Diagnostics.StackTrace(e, true).GetFrame(0).GetFileLineNumber();

				linenum = Convert.ToInt32(e.StackTrace.Substring(e.StackTrace.LastIndexOf(' ')));
				Trace.WriteLine(linenum + ": " + RawDepthHeight + "," + RawDepthWidth + "," + row + "," + col);


				throw;
			}


			return outputImage;
		}

		/**********************RefractoionAndToFCorrection***********************/
		// <summary>
		//  REFRACTION & TOF CORRECTION
		// </summary>
		// <remarks>
		//	1. Refraction Correction for Depth images in water
		//		Basically Ray Tracing each pixel from a pinhole camera
		//		Function call takes image as input and returns the refraction corrected image
		//		assuming the argument image is already in uint16 with pixesl as depth values
		//				\
		//				 \	air
		//		 ----------------------
		//				  |
		//				  |	pespex
		//		 ----------------------
		//				   \
		//					\	water
		//	2. ToF correction
		//		Kinect Calculates distances using Time of Flight principal in AIR
		//		Since we are taking it underwater, therefore the speed of light will change
		//		from that of in air.
		//		This function also caters for the difference of medium; by recalculating the differences
		//		of distance by change of medium
		//		Calculation is done INSIDE the refraction correction function; no explicit function called.
		//	3. Preferably called after median filtering; but can be called earlier aswell.
		// 	4.	The output image is a byte 1-D array (ushort)
		//	5. Variable Definitions:
		//		rzx and rzy: Polar (r,theta) ray along z-axis (rz) with x-projection (rzx) and y-projection(rzy)
		// </remarks>
		/// <param name="inputImage"> The input Image</param>
		/// The input image is a byte 1-D array (ushort) .
		/// <param name="filter_width"> Median Fiter width</param>
		/// <param name="filter_height">Median Filter heignt</param>
		/// <param name="depthImagePixels"> the Original depth image data</param>
		/// <returns> Depth image ushort array </returns>
		///
		public ushort[] RefractoionAndToFCorrection(ushort[] depthImagePixels)
		{
			// Variable definitions
			// note that angles will have to be double and not ushort as angles will also have negative values
			// ushort cannot accept negative values.
			double[][] thetaAir = new double[RawDepthHeight][];
			double[][] phiAir = new double[RawDepthHeight][];
			double[][] rzx_water = new double[RawDepthHeight][];
			double[][] rzy_water = new double[RawDepthHeight][];
			double[][] rzx_perspex = new double[RawDepthHeight][];
			double[][] rzy_perspex = new double[RawDepthHeight][];
			double[][] theta2 = new double[RawDepthHeight][];
			double[][] phi2 = new double[RawDepthHeight][];
			ushort[] depthCorrectedpixels = new ushort[RawDepthHeight];
			double[][] dShift = new double[RawDepthHeight][];

			var incidentAirRays = Tuple.Create(thetaAir, phiAir);
			var anglesPerspex = Tuple.Create(theta2, phi2);
			var anglesWater = Tuple.Create( theta2, phi2 );
			var rayLengthsPerspex = Tuple.Create(rzx_perspex, rzy_perspex);
			var rayLengthsWater = Tuple.Create( rzx_perspex, rzy_perspex );

			// ----------- The loops ----------- //
			//A.  FOR AIR: Get Incident ray Matrix, theta and phi for all 512x424 pixels
			incidentAirRays = incidentRays(depthImagePixels);

			//B. FOR AIR -> PERSPEX: Calculate new refracted angles
			/// Get Incident ray Matrix, theta and phi for all 512x424 pixels
			anglesPerspex = calculateRefractedAngles(incidentAirRays.Item1, incidentAirRays.Item2, nIndexAir, nIndexPerspex);
			rayLengthsPerspex = refractRays(depthImagePixels, anglesPerspex.Item1, anglesPerspex.Item2, perspex);

			//C. FOR PERSPEX -> WATER: calculate new refracted angles
			// Get Incident ray Matrix, theta and phi for all 512x424 pixels
			anglesWater = calculateRefractedAngles( anglesPerspex.Item1, anglesPerspex.Item2, nIndexPerspex, nIndexWater);
			rayLengthsWater = refractRays(depthImagePixels, anglesWater.Item1, anglesWater.Item2, water);

			// D. rzy_water is the hypotenuse(b) to calculate the dShift variable
			// rzx_water is its projection in a plane parallel to the depth measured.So that is one used here (?)
			// To calculate the dShift matrix for all the image pixels(depth measurements):
			dShift = calculateDistanceError(depthImagePixels, rayLengthsWater.Item1, rayLengthsWater.Item2);
			
			// corrected_image = rzy_water;
			double[][] corrected_image = rayLengthsWater.Item2;

			depthCorrectedpixels = TwoDto1D(corrected_image);

			// return the refraction corrected and Tof adjusted depth pixels
			// returns ushort 1D vector as input is the same
			return depthCorrectedpixels;
		}

		/***********************incidentRays**********************/
		// <summary>
		// Get Incident ray Matrix, theta and phi for all 512,424 pixels
		// </summary>
		/// <param name="inputImage"> The input image is a byte 1-D array (ushort) </param>
		/// <returns> Tuple with 2 ushort 1-D matrices reperesenting Item1: thetaAir and Item2: phiAir </returns>
		public Tuple<double[][], double[][]> incidentRays(ushort[] inputImage)
		{
			// init row and col for 0 index
			int col = 0;
			int row = 0;

			// initialize 2D matrices
			double[][] thetaAir = new double [RawDepthHeight][];
			double[][] phiAir = new double[RawDepthHeight][];

			// Vertical FOV is actually image rows
			for (double i = (-1) * VFOVcenter; i < VFOVcenter && row < RawDepthHeight; i += verticalAngleResolution)
			{
				// skip the center 0 value as axis is between the two center pixels
				if (i == 0)
				{
					continue;
				}

				thetaAir[row] = new double[RawDepthWidth];
				phiAir[row] = new double[RawDepthWidth];
				// Horizontal FOV is actually image Columns
				for (double j = (-1) * HFOVcenter; j < HFOVcenter && col < RawDepthWidth; j += horizongtalAngleResolution)
				{
					// skip the center 0 value as axis is between the two center pixels
					if (j == 0)
					{
						continue;
					}

					thetaAir[row][col] = Math.Round( j, 4);
					phiAir[row][col] = Math.Round( i, 4);

					col += 1;           // row counter
				}

				// reset col
				col = 0;

				row += 1;               // col counter
			}
			var incidentRays = Tuple.Create(thetaAir, phiAir);
			return incidentRays;
		}

		/***********************calculateRefractedAngles**********************/
		// <summary>
		// Snell's Law to calculate refraction angles based on material refraction angles
		// </summary>
		/// <param name="theta1"> incident theta matroix </param>
		/// <param name="phi1"> incident phi matroix </param>
		/// <param name="nIndexI"> Incident medium refraction index </param>
		/// <param name="nIndexR"> Refracted medium refraction index </param>
		// <returns> 2 ushort 1-D matrices reperesenting refracted Theta2 and Phi2 </returns>
		public Tuple<double[][], double[][]> calculateRefractedAngles(double[][] theta1, double[][] phi1, double nIndexI, double nIndexR)
		{
			// initialize theta and phi variables
			double[][] theta2 = new double[RawDepthHeight][];
			double[][] phi2 = new double[RawDepthHeight][];
			int row;

			for (row = 0; row < RawDepthHeight; row++)
			{
				theta2[row] = new double[RawDepthWidth];
				phi2[row] = new double[RawDepthWidth];
				for (int col = 0; col < RawDepthWidth; col++)
				{
					// note: Asin and Sin return double.. so might be an error? Remove when done
					// rounding off to 4 digits
					theta2[row][col] = Math.Round( Math.Asin((nIndexI / nIndexR) * (Math.Sin(theta1[row][col]))), 4);
					phi2[row][col] = Math.Round( Math.Asin((nIndexI / nIndexR) * (Math.Sin(phi1[row][col]))), 4);
				}
			}

			var angles = Tuple.Create(theta2, phi2);
			return angles;
		}

		/************************refractRays*********************/
		// <summary>
		// Get Incident ray Matrix, theta and phi for all 512x424 pixels
		// </summary>
		/// <param name="refractedTheta"> The input image is a byte 1-D array (ushort) </param>
		/// <param name="refractedPhi"> The input image is a byte 1-D array (ushort) </param>
		/// <param name="flag"> String to specify if refracting for perspex or water  </param>
		/// <param name="rzx"> vector lenths in X-Axis in a 1-D array  </param>
		/// <param name="rzy"> vector lenths in Y-Axis in a 1-D array  </param>
		/// <returns> 2 ushort 1-D matrices reperesenting updated RZX and RZY </returns>
		public Tuple<double[][], double[][]> refractRays(ushort[] depthImagePixels, double[][] refractedTheta, double[][] refractedPhi, int flag)
		{

			// *************************************************************//
			/*----------------- Time of Flight correction -------------------*/
			// check the function call option

			double[][] dist_uncorrected = new double[RawDepthHeight][];
			double[][] depth_img_updated = new double[RawDepthHeight][];
			double[][] dist = new double[RawDepthHeight][];
			double[][] depthImagePixels2D = new double[RawDepthHeight][];
			double[][] rzx_perspex = new double[RawDepthHeight][];
			double[][] rzy_perspex = new double[RawDepthHeight][];
			double[][] time_orig = new double[RawDepthHeight][];
			double[][] t_water = new double[RawDepthHeight][];
			double[][] d_water = new double[RawDepthHeight][];
			double[][] rzx_water = new double[RawDepthHeight][];
			double[][] rzy_water = new double[RawDepthHeight][];
			double[][] rzx_water_uncorrected = new double[RawDepthHeight][];
			double[][] rzy_water_uncorrected = new double[RawDepthHeight][];
			double[][] rayLengthZX = new double[RawDepthHeight][];
			double[][] rayLengthZY = new double[RawDepthHeight][];


			double t_air;
			double t_perspex;


			// converting the 1D vector to 2D matrix for refraction/TOF correction
			depthImagePixels2D = OneDto2D(depthImagePixels);

			// perspex:
			// distance is just thickness of housing (2mm)
			if (flag == 1)
			{
				for (int i = 0; i < RawDepthHeight; i++)
				{
					dist[i] = new double[RawDepthWidth];
					for (int j = 0; j < RawDepthWidth; j++)
					{
						dist[i][j] = dist_perspex;
					}
				}
			}
			// ~~~~~~~~~ water ~~~~~~~~~~~~~
			// TIME OF FLIGHT CORRECTION BEFORE CALCULATING NEW DISTANCES
			// recalculate the distances matrix to adjust the speed of light in water (ToF calculation adjustment)
			else if (flag == 2)
			{
				for (int i = 0; i < RawDepthHeight; i++)
				{
					dist_uncorrected[i] = new double[RawDepthWidth];
					// depthImagePixels2D[i] = new double[RawDepthWidth];
					for (int j = 0; j < RawDepthWidth; j++)
					{
						dist_uncorrected[i][j] = depthImagePixels2D[i][j] - (dist_air + dist_perspex);
					}
				}

				// calculating original time of each pixel depth measurement
				for (int i = 0; i < RawDepthHeight; i++)
				{
					time_orig[i] = new double[RawDepthWidth];
					for (int j = 0; j < RawDepthWidth; j++)
					{
						time_orig[i][j] = (double)(depthImagePixels2D[i][j] / c_air);
					}

				}

				// calculating time of flight in water for each pixel
				t_air = dist_air / c_air;
				t_perspex = dist_perspex / c_perspex;
				for (int i = 0; i < RawDepthHeight; i++)
				{
					t_water[i] = new double[RawDepthWidth];
					for (int j = 0; j < RawDepthWidth; j++)
					{
						t_water[i][j] = time_orig[i][j] - (t_air + t_perspex);
					}

				}

				// calcualting distance based on the calculated time in water
				// calcualting updated depth image as a sum of the three distance calculated
				for (int i = 0; i < RawDepthHeight; i++)
				{
					d_water[i] = new double[RawDepthWidth];
					depth_img_updated[i] = new double[RawDepthWidth];
					dist[i] = new double[RawDepthWidth];
					for (int j = 0; j < RawDepthWidth; j++)
					{
						d_water[i][j] = t_water[i][j] * c_water;
						depth_img_updated[i][j] = d_water[i][j] + dist_air + dist_perspex;
						dist[i][j] = depth_img_updated[i][j] - (dist_air + dist_perspex);
					}

				}
			}

			// loop format: L to R
			// keeping rows same; parse all columns

			for (int i = 0; i < RawDepthHeight; i++)
			{
				rzx_perspex[i] = new double[RawDepthWidth];
				rzy_perspex[i] = new double[RawDepthWidth];
				rzx_water[i] = new double[RawDepthWidth];
				rzy_water[i] = new double[RawDepthWidth];
				rzx_water_uncorrected[i] = new double[RawDepthWidth];
				rzy_water_uncorrected[i] = new double[RawDepthWidth];

				for (int j = 0; j < RawDepthWidth; j++)
				{
					if (flag == 1)
					{
						rzx_perspex[i][j] = Math.Round( dist[i][j] / Math.Cos(refractedTheta[i][j]), 4);
						rzy_perspex[i][j] = Math.Round( rzx_perspex[i][j] / Math.Cos(refractedPhi[i][j]), 4);
					}
					else if (flag == 2)
					{
						// if water; then calculate
						rzx_water[i][j] = Math.Round( dist[i][j] / Math.Cos(refractedTheta[i][j]), 4);
						rzy_water[i][j] = Math.Round( rzx_water[i][j] / Math.Cos(refractedPhi[i][j]), 4);
						// rzx_water_uncorrected[i][j] = dist_uncorrected[i][j] / Math.Cos(refractedTheta[i][j]);
						// rzy_water_uncorrected[i][j] = rzx_water_uncorrected[i][j] / Math.Cos(refractedPhi[i][j]);
					}
				}
			}

			if (flag == 1)
			{
				// perspex
				rayLengthZX = rzx_perspex;
				rayLengthZY = rzy_perspex;
			}
			else if (flag == 2)
			{
				// water
				rayLengthZX = rzx_water;
				rayLengthZY = rzy_water;
			}

			// return the updated refracted angles
			//var angles = Tuple.Create(theta2, phi2);
			//return angles;

			var rayLengths = Tuple.Create(rayLengthZX, rayLengthZY);
			return rayLengths;
			/*----------------- END ToF correction -------------------------*/
			// *************************************************************//

		}

		/************************calculateDistanceError*********************/
		public double[][] calculateDistanceError(ushort[] depth_img, double[][] rzx_water, double[][] rzy_water)
		{
			double[][] dShift = new double[RawDepthHeight][];
			double[][] temp = new double[RawDepthHeight][];

			//convert to 2D
			temp = OneDto2D(depth_img);

			for ( int row = 1; row < RawDepthHeight; row++)
			{
				dShift[row] = new double[RawDepthWidth];
				for ( int col = 1; col < RawDepthWidth; col++)
				{
					dShift[row][col] = rzy_water[row][col] - temp[row][col];
				}
			}

			return dShift;
		}

		// *****************************OneDto2D********************************//
		/// <summary>
		/// Converts 1D vector to 2D matrix for processing refraction correction
		/// </summary>
		/// <param name="input1Dmatrix">input 1D raw depth matrix</param>
		/// <returns>ushort coverted 2D matrix. No need to convert to double (?)</returns>
		public double[][] OneDto2D(ushort[] input1Dmatrix)
		{
			double[][] converted2DMatrix = new double[RawDepthHeight][];
			int index = 0;
			for (int row=0; row < RawDepthHeight; row++)
			{
				converted2DMatrix[row] = new double[RawDepthWidth];
				for (int col=0; col < RawDepthWidth; col++)
				{
					converted2DMatrix[row][col] = (double)(input1Dmatrix[index]);
					index++;
				}
			}
			return converted2DMatrix;
		}

		// *****************************TwoDto1D********************************//
		/// <summary>
		/// Converts 2D matrix to 1D vector before returning back to Kinect Fusion
		/// </summary>
		/// <param name="TwoDmatrix">Input processed 2D matrix</param>
		/// <returns>ushort coverted 1D matrix. No need to convert to double (?)</returns>
		public ushort[] TwoDto1D (double[][] TwoDmatrix)
		{
			ushort[] OneDvector = new ushort[RawDepthWidth*RawDepthHeight];
			int index = 0;

			for (int row = 0; row < RawDepthHeight; row++)
			{
				for (int col = 0; col < RawDepthWidth; col++)
				{
					if (TwoDmatrix[row][col] < 0)
					{
						TwoDmatrix[row][col] = 0;
					}
					else
					{
						// round off to a whole number before converting to ushort
						OneDvector[index] = (ushort)(Math.Round(TwoDmatrix[row][col], 0));
					}
					index++;
				}
			}
			return OneDvector;
		}
		// *************************************************************//

		/**********************getValue***********************/
		/// <param name="row">Row</param>
		/// <param name="col">Col</param>
		/// <returns>ushort value</returns>
		public ushort getValue( int row, int col )
		{
			ushort val = 0;
			if ( row < 0 )
				row = 0;
			else if ( row >= RawDepthHeight )
				row = RawDepthHeight - 1;

			if ( col < 0 )
				col = 0;
			else if ( col >= RawDepthWidth )
				col = RawDepthWidth - 1;

			val = m_data[row][col];
			return val;
		}

		/**********************setValue***********************/
		/// <summary>
		/// Set value in matrix at Row Col index
		/// </summary>
		/// <param name="row"></param>
		/// <param name="col"></param>
		/// <param name="value"></param>
		public void setValue( int row, int col, ushort value )
		{
			tmpMatrix[row][col] = value;

			//return m_data;
		}

		/******************** Camera Calibration and Undistortion ******************* */
		/// <summary>
		/// The distortion parameters calculated to be incorporated in the Kinect Fusion code
		/// Cakibration to be added before filtering, on the depth image data
		/// Inspired from https://stackoverflow.com/questions/12620025/barrel-distortion-correction-algorithm-to-correct-fisheye-lens-failing-to-impl
		/// and: http://www.4pi.org/panoramas.html
		/// </summary>
		/// <param inputImage="1D depth image from Kinect"></param>
		/// 
		public ushort[] cameraCalibration1D( ushort[] inputImage )
		{
			// Initializing the o/p image
			ushort[] tempCal = new ushort[inputImage.Length];

			//double coeff1 = 1.5807;		// Calculated by GML Toolbox
			//double coeff2 = -1.8271;
			//double coeff3 = 0.204611;
			//double coeff4 = -0.000325;

			// parameters for correction
			// Normalizing: 1-1/param_calculated (to bring it between 0 and 1)

			double paramA = 0.333; // affects only the outermost pixels of the image
			double paramB = -0.5; // most cases only require b optimization
			double paramC = 0.1; // most uniform correction
			double paramD = 1.0 - paramA - paramB - paramC; // describes the linear scaling of the image
			//double paramD = 0.85;

			for ( int x = 0 ; x < RawDepthWidth ; x++ ) //512
			{
				for ( int y = 0 ; y < RawDepthHeight ; y++ ) //424
				{
					int d = Math.Min( RawDepthWidth, RawDepthHeight) / 2;    // radius of the circle = 424/2 = 212

					// center of dst image
					double centerX = (RawDepthWidth) / 2.0;
					double centerY = (RawDepthHeight) / 2.0;

					// cartesian coordinates of the destination point (relative to the centre of the image)
					double deltaX = (x - centerX) / d;
					double deltaY = (y - centerY) / d;

					// distance or radius of dst image
					double dstR = Math.Sqrt( deltaX * deltaX + deltaY * deltaY );

					// distance or radius of src image (with formula)
					double srcR = (paramA * dstR * dstR * dstR + paramB * dstR * dstR + paramC * dstR + paramD) * dstR;

					// comparing old and new distance to get factor
					double factor = Math.Abs( dstR / srcR );

					// coordinates in source image
					double srcXd = centerX + (deltaX * factor * d);
					double srcYd = centerY + (deltaY * factor * d);

					// no interpolation yet (just nearest point)
					//int srcX = (int)srcXd;
					//int srcY = (int)srcYd;
					int srcX = (int)Math.Round(srcXd);
					int srcY = (int)Math.Round(srcYd);

					if ( srcX >= 0 && srcY >= 0 && srcX < RawDepthWidth && srcY < RawDepthHeight)
					{
						int dstPos = y * RawDepthWidth + x;
						tempCal[dstPos] = inputImage[srcY * RawDepthWidth + srcX];
					}
				}
			}
			return tempCal;
		}
		
	} //End Class
} // End namespace
 