using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Media.Imaging;
using System.Drawing.Drawing2D;
using System.Runtime.InteropServices;


/****************************************
 * Original Source code
 * http://www.helviojunior.com.br/fotografia/barrel-and-pincushion-distortion/
 * Author: Helvio Junior
 ***************************************/
namespace HelvioJunior
{
    class Program
    {
   //     static void Main(string[] args)
   //     {

			//Bitmap bmp = (Bitmap)Bitmap.FromFile("test.jpg");
   //         bmp = BarrelDistortion(bmp, 0.1, false, Color.White);
   //         bmp.Save("new_test.jpg");
   //         bmp.Dispose();

   //         Console.WriteLine("Pressione ENTER para finalizar");
   //         Console.ReadLine();
   //     }

        static public Bitmap BarrelDistortion(Bitmap StartImage, double factor)
        {
            return BarrelDistortion(StartImage, factor, true, Color.Transparent);
        }

        static public Bitmap BarrelDistortion(Bitmap sourceImage, double factor, Boolean autoCrop, Color backgroundColor)
        {
            Bitmap StartImage = null;
            BitmapData srcBitmapData = null;
            Byte[] srcPixels = null;
            Byte[] dstPixels = null;
            Bitmap NewImage = null;
            BitmapData dstBitmapData = null;

            try
            {

				//// Verifica se bpp (Bits Per Pixel) é 8, 24, ou 32
				// int Depth = System.Drawing.Bitmap.GetPixelFormatSize(sourceImage.PixelFormat);
				int Depth = 32;
                //if (Depth != 8 && Depth != 24 && Depth != 32)
                //{
                //    throw new ArgumentException("Only 8, 24 and 32 bpp images are supported.");
                //}

				// Retrieves counts of color components
				int cCount = Depth / 8;

                Size baseSize = new Size(512, 424);

				////Verifies if it is a low-key image and needs to resize to improve quality
				////e não gerar serrilhamento da imagem
				//Int32 maxSize = Math.Max(sourceImage.Width, sourceImage.Height);
    //            if (maxSize < 3000)
    //            {
    //                float percent = 3000F / (float)maxSize;
    //                baseSize = new Size((Int32)((float)sourceImage.Width * percent), (Int32)((float)sourceImage.Height * percent));
    //            }

                StartImage = new Bitmap(baseSize.Width, baseSize.Height, sourceImage.PixelFormat);
                StartImage.SetResolution(sourceImage.HorizontalResolution, sourceImage.VerticalResolution);

                // Create drawing object and white background
				Graphics g = Graphics.FromImage(StartImage);
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                g.DrawImage(sourceImage, new Rectangle(-1, -1, baseSize.Width + 1, baseSize.Height + 1), 0, 0, sourceImage.Width, sourceImage.Height, GraphicsUnit.Pixel);
                g.Dispose();


				// Blocks the source image and copies it to the byte array and releases the source image
				srcBitmapData = StartImage.LockBits(new Rectangle(0, 0, StartImage.Width, StartImage.Height), ImageLockMode.ReadOnly, StartImage.PixelFormat);
				srcPixels = new byte[StartImage.Width * StartImage.Height * (Depth / 8)];
				Marshal.Copy( srcBitmapData.Scan0, srcPixels, 0, srcPixels.Length );
				StartImage.UnlockBits( srcBitmapData );
				srcBitmapData = null;

				//Creates the byte array of the destination image
				dstPixels = new Byte[srcPixels.Length];

				//Fill the entire frame with the selected background color
				Int32 index = ((1 * StartImage.Width) + 1) * cCount; //index = ((Y * Width) + X) * cCount
                do
                {
					if ( Depth == 32 ) // Para 32 bpp define Red, Green, Blue e Alpha
					{
						dstPixels[index++] = backgroundColor.B;
						dstPixels[index++] = backgroundColor.G;
						dstPixels[index++] = backgroundColor.R;
						dstPixels[index++] = backgroundColor.A; // a
					}
					if ( Depth == 24 ) // Para 24 bpp define Red, Green e Blue
					{
						dstPixels[index++] = backgroundColor.B;
						dstPixels[index++] = backgroundColor.G;
						dstPixels[index++] = backgroundColor.R;
					}
					if ( Depth == 8 )
						//For 8 bpp sets the value of the color( Red, Green and Blue as being the same thing )
					{
						dstPixels[index++] = backgroundColor.B;
                    }

                } while (index < srcPixels.Length);


				//Calculates the maximum possible amplitude for the image and multiplies by the desired factor
				double amp = 0;
                double ang = Math.PI * 0.5;
                for (Int32 a = 0; a < StartImage.Height; a++)
                {
                    int y = (int)((StartImage.Height / 2) - amp * Math.Sin(ang));
                    if ((y < 0) || (y > StartImage.Height))
                        break;
                    amp = a;
                }
                amp = (amp - 2) * (factor < -1 ? -1 : (factor > 1 ? 1 : factor));


				//Defines variables that calculates the cutoff points (if any)
				Int32 x1, y1, x2, y2;
                x1 = StartImage.Width;
                y1 = StartImage.Height;
                x2 = 0;
                y2 = 0;

				//Copy pixel by pixel to new positions
				index = ((1 * StartImage.Width) + 1) * cCount;
                do
                {

                    Int32 y = (Int32)((index / cCount) / StartImage.Width);
                    Int32 x = (index / cCount) - (y * StartImage.Width);

                    Point pt = NewPoint(new Point(x, y), StartImage.Width, StartImage.Height, amp, factor < 0);

					//Values ​​for crop
					if ( factor >= 0)
                    {
                        if (x == StartImage.Width / 2)
                        {
                            if (pt.Y < y1)
                                y1 = pt.Y;

                            if (pt.Y > y2)
                                y2 = pt.Y;
                        }

                        if (y == StartImage.Height / 2)
                        {
                            if (pt.X < x1)
                                x1 = pt.X;

                            if (pt.X > x2)
                                x2 = pt.X;
                        }
                    }
                    else
                    {
                        if ((x == 1) && (y == 1))
                        {
                            y1 = pt.Y;
                            x1 = pt.X;
                        }

                        if ((x == StartImage.Width - 1) && (y == StartImage.Height - 1))
                        {
                            y2 = pt.Y;
                            x2 = pt.X;
                        }
                    }

					//Byte index where the pixel will be applied
					Int32 dstIndex = ((pt.Y * StartImage.Width) + pt.X) * cCount;

                    if (Depth == 32)
                    {
                        dstPixels[dstIndex] = srcPixels[index++];
                        dstPixels[dstIndex + 1] = srcPixels[index++];
                        dstPixels[dstIndex + 2] = srcPixels[index++];
                        dstPixels[dstIndex + 3] = srcPixels[index++]; // a
                    }
                    if (Depth == 24)
                    {
                        dstPixels[dstIndex] = srcPixels[index++];
                        dstPixels[dstIndex + 1] = srcPixels[index++];
                        dstPixels[dstIndex + 2] = srcPixels[index++];
                    }
                    if (Depth == 8)
                    {
                        dstPixels[dstIndex] = srcPixels[index++];
                    }

                } while (index < srcPixels.Length);

				//Creates the new image based on the previously created byte array
				NewImage = new Bitmap(StartImage.Width, StartImage.Height, StartImage.PixelFormat);
                NewImage.SetResolution(StartImage.HorizontalResolution, StartImage.VerticalResolution);
				dstBitmapData = NewImage.LockBits( new Rectangle( 0, 0, StartImage.Width, StartImage.Height ), ImageLockMode.WriteOnly, StartImage.PixelFormat );
				Marshal.Copy( dstPixels, 0, dstBitmapData.Scan0, dstPixels.Length );
				NewImage.UnlockBits( dstBitmapData );


				//// For visualization effect, draw the square where the cut will be made
				//Graphics g2 = Graphics.FromImage(NewImage);
				//g2.SmoothingMode = SmoothingMode.AntiAlias;
				//g2.InterpolationMode = InterpolationMode.HighQualityBicubic;
				//g2.PixelOffsetMode = PixelOffsetMode.HighQuality;
				//g2.DrawRectangle(new Pen(new SolidBrush(Color.Red), 3), new Rectangle(x1, y1, x2 - x1, y2 - y1));
				//g2.Dispose();

				//It generates the final image, with crop or coo real resizing
				Bitmap FinalImage = new Bitmap(sourceImage.Width, sourceImage.Height, StartImage.PixelFormat);
                NewImage.SetResolution(StartImage.HorizontalResolution, StartImage.VerticalResolution);

                Graphics g1 = Graphics.FromImage(FinalImage);
                g1.SmoothingMode = SmoothingMode.AntiAlias;
                g1.InterpolationMode = InterpolationMode.HighQualityBicubic;
                g1.PixelOffsetMode = PixelOffsetMode.HighQuality;

				//Performs cutting if automatic cutting is enabled and cutting is required
				if ( (autoCrop) && ((x1 > 0) || (y1 > 0) || (x2 < NewImage.Height) || (y2 < NewImage.Height)))
                {
                    Rectangle cropRect = new Rectangle(x1, y1, x2 - x1, y2 - y1);
                    g1.DrawImage(NewImage, new Rectangle(-1, -1, FinalImage.Width + 1, FinalImage.Height + 1), cropRect.X, cropRect.Y, cropRect.Width, cropRect.Height, GraphicsUnit.Pixel);
                }
                else
                {
                    g1.DrawImage(NewImage, new Rectangle(-1, -1, FinalImage.Width + 1, FinalImage.Height + 1), 0, 0, NewImage.Width, NewImage.Height, GraphicsUnit.Pixel);
                }

                g1.Dispose();
                g1 = null;

                NewImage = null;
                return FinalImage;
            }
            finally
            {
                //srcBitmapData = null;
                srcPixels = null;
                dstPixels = null;
                //dstBitmapData = null;
            }

        }

        private static Point NewPoint(Point AtualPoint, Int32 Width, Int32 Height, double Aplitude, Boolean inverse)
        {
            Point uP = AtualPoint;

            int pY, pX;
            double aY, aX;

            aY = aX = 0;

            double angX = Math.PI * 1 * (double)uP.X / (double)Width;
            double caX = Aplitude * ((((double)Height / 2F) - (double)uP.Y) / ((double)Height / 2F));

            double angY = Math.PI * 1 * (double)uP.Y / (double)Height;
            double caY = Aplitude * ((((double)Width / 2F) - (double)uP.X) / ((double)Width / 2F));

            if (inverse)
            {
                double iAng = Math.PI * -1 * 0.5;
                aX = (caX * Math.Sin(iAng));
                aY = (caY * Math.Sin(iAng));
            }

            pY = (int)(uP.Y + aX + caX * Math.Sin(angX));
            pX = (int)(uP.X + aY + caY * Math.Sin(angY));

            return new Point(pX, pY);

        }

    }
}

