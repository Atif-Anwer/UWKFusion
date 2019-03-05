# UNDERWATER 3D SCENE RECONSTRUCTION USING KINECT V2 #

### What is this repository for? ###
The repository is the front end GUI and developed filters for 3D scene reconstruction using Kinect Fusion for Kinect v2 on depth data acquired underwater. The application has been developed in C# with a front end in XAML.
The designed filters implemented in the GUI are as follows:

1. Kinect camera calibration (RGB and NIR) for underwater application
2. Median filtering on depth data
3. Time of flight correction algorithm
4. Refraction correction algorithm

Details of the working of the above algorithms can be found from our publication:
[UNDERWATER 3D SCENE RECONSTRUCTION USING KINECT V2 TIME OF FLIGHT CAMERA](https://doi.org/10.1109/ACCESS.2017.2733003)

### Video about the Research  ###

[![Underwater 3D Scene Reconstruction Using Kinect v2, with Refraction and ToF Correction](http://i.imgur.com/CdMpSJEm.png)](https://www.youtube.com/watch?v=E5GNbEN16uQ "Underwater 3D Scene Reconstruction Using Kinect v2, with Refraction and ToF Correction")


### Dataset of Kinect Data taken underwater ###

A complete dataset has been acquired consisting of various objects scanned underwater. The data from Kinect’s RGB and IR cameras was captured alongside the generated point cloud by KinectToF and saved in Microsoft’s eXtended Event File (XEF) file format that can be used with Kinect Studio application. The dataset s publicly available under GNU GPL 3.0 license and can be downloaded from the link below:

* [Kinect Underwater Dataset](http://bit.ly/3DUWK)

### Hardware and Software requirements? ###

The following software’s are required for use
* [Kinect for Windows SDK 2.0](https://www.microsoft.com/en-us/download/details.aspx?id=44561)
* Visual Studio 2015 (code has not been tested with VS 2017 as yet, but it should work)
* [Dot Net framework 4.5 or greater](https://www.microsoft.com/en-in/download/details.aspx?id=30653)
* Windows 8.1 or greater (Developed on Windows 10, but should work for Windows 8 aswell)

Following are the recommended hardware requirements for testing/using the application:
* Core i7 (Ivy Bridge or greater)
* 8 GB Memory (16 GB preferrable if using the dataset)
* USB 3.0 controller dedicated to the Kinect for Windows v2 sensor*
* DX11 capable graphics adapter (Nvidia or AMD)
* A Microsoft Kinect v2 sensor (Xbox One or Kinect v2 for windows)
* Microsoft Kinect for Windows Adapter (Xbox One)
* SSD harddrive is preferred if using the dataset, for optimum performance.

Note: This application is not meant to run with Kinect for Xbox 360 or Kinect V1

### Kinect Casing Assembly ###

![Casing Assembly Design](http://i.imgur.com/3cbA1IKl.jpg)

### Some sample RGB images from the dataset ###
| Dataset | RGB Image | Description |
| --- | --- | --- |
| 3D Models of Objects | ![STL Models](http://i.imgur.com/AvjYyYDt.png) | 3D printed STL files, also used as ground truth for alignment error map calculation |
| Swimming Pool Wall | ![Swimming Pool wall](http://i.imgur.com/axdiiFJt.png) | A flat and tiled swimming pool wall. Provides good data for RGB and IR camera calibration underwater |
| Swimming Pool Objects | ![Objects](http://i.imgur.com/ubRYh33t.png) | Objects were placed in water held down by weights. Data captured moves the Kinect around the objects  |
| Offshore Lab - Low Light | ![Low Light](http://i.imgur.com/viclDRzt.png) | Images of a few objects and a checker board were taken in the offshore lab, in low light |

There are more XEF files with several scanes of objects underwater. For more details, please download the [Kinect Underwater Dataset](https://1drv.ms/f/s!AphLSHrbZ957xh_9K93h8ABPQXUb)

### Using the Dataset with the application ###

The [Kinect Underwater Dataset](http://bit.ly/3DUWK) is the raw data captured with Kinect Studio 2.0 in the proprietary XEF file format. Kinect Studio is bundled in the SDK 2.0 and can be used to open the dataset files. Once opened, Kinect Studio emulates the Kinect hardware and plays back the recorded files as if an actual Kinect Device is attached to the USB port. 

For improving performance, please note the following (quoted from [MSDN](https://msdn.microsoft.com/en-us/library/hh855390.aspx) ):
> Recording and playing back Kinect data in Kinect Studio requires sufficient computer resources. If throughput of the data is not high enough, you can experience dropped data frames. To get good performance when recording data, you need space on your hard drive, a reasonably fast CPU, and extra RAM. Playback requires a file for playback, and KinectStudio must be connected to a running Kinect application. This is because you play back recorded data in the Kinect application as if the recorded data were coming from the actual sensor.
>The following tips will help you get the best performance:
>* Make sure that your computer has sufficient RAM to run your operating system.
>* Use a computer with a fast hard disk drive (HDD). The HDD should have plenty of empty space available and should be reasonably unfragmented.
>* Run as few other applications as possible.
>* Close any Kinect Studio viewing windows that you aren't using.
>* If you use a network location to save temporary files, you may experience dropped frames. To avoid this problem, change the temporary file location (under Tools > Options) to a local path.'

### IEEE Transaction ###

The Research article can be downloaded from: [IEEE ACCESS](https://doi.org/10.1109/ACCESS.2017.2733003)

### Citation ###

The code and dataset are available for testing or experimenting. However, we would appreciate if you can cite our paper while publishing any work online, in any conference of journal. 
For citation, please use the following bibtext entry:
```
@article{Anwer2017,
  doi = {10.1109/access.2017.2733003},
  url = {https://doi.org/10.1109/access.2017.2733003},
  year  = {2017},
  publisher = {Institute of Electrical and Electronics Engineers ({IEEE})},
  pages = {1--1},
  author = {Atif Anwer and Syed Saad Azhar Ali and Amjad Khan and Fabrice Meriaudeau},
  title = {Underwater 3D Scene Reconstruction Using Kinect v2 Based on Physical Models for Refraction and Time of Flight Correction},
  journal = {{IEEE} Access}
}
```

### Contact ###

For any query, please feel free to contact the following authors:

* Atif Anwer[Google Scholar](https://scholar.google.com/citations?user=qsP3e2kAAAAJ)| [Website](https://www.atifanwer.xyz)
* [Syed Saad Azhar Ali](https://scholar.google.com/citations?user=x3GCOQMAAAAJ)
* [Amjad Khan](https://scholar.google.com/citations?user=WEVTyZsAAAAJ)
* [Fabrice Mériaudeau](https://scholar.google.com/citations?user=tNttgvEAAAAJ)
