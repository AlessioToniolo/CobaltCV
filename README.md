# CobaltCV
![CobaltCV](https://github.com/AlessioToniolo/FTC-EOCV-Ultimate-Goal/blob/master/circlecobaltimage.png)

## Welcome
This is the official repository for CobaltCV the new computer vision library for FTC! Inspired by past vision libraries including [DogeCV](https://github.com/dogecv/DogeCV)
and EasyOpenCV, this is a field-ready computer vision library with game support starting at Ultimate Goal 2020-2021. Designed for beginner
programmers and on, there is finally no intense java expierience required to use computer vision on your robot! We take the sweat out of
vision with sample autonomous programs and premade pipelines along with compatibility for using a phone camera, control hub, and external
webcams! Written based off of [EasyOpenCV](https://github.com/OpenFTC/EasyOpenCV), code is highly functional and established off of a reliable source. To get started, follow the steps below.

## Getting Started
To get started with using **CobaltCV**, you have two options:
1. Option 1: Download repository
* Copy the link of the repository then using Git bash in the folder where you want your repository, type in `git clone https://github.com/AlessioToniolo/FTC-EOCV-Ultimate-Goal`
* Open the folder in android studio, then navigate to the teamcode folder and you should find two folders with files inside them: vision and drive
* Once you have seen this, you are all set! You can add any other code files for your robot as needed
2. Option 2: Add to an existing repository
* Open your team's sdk project, and navigate to *build.common.gradle*
* Add the code `jcenter()` to `repositories`
* Navigate to *build.gradle* for teamcode and add `implementation 'org.openftc:easyopencv:1.4.1` to dependencies
* From there add the [vision](https://github.com/AlessioToniolo/FTC-EOCV-Ultimate-Goal/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision) files to your project
* Use as needed! If you would like to use our vision samples, download them from [drive](https://github.com/AlessioToniolo/FTC-EOCV-Ultimate-Goal/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive) in teamcode
* You are finished!

### Adding OpenCV to your robot controller
In order to use OpenCV, you must download this file and add it to your robot controller files, whether that would be a control hub or robot phone.
**https://github.com/AlessioToniolo/FTC-EOCV-Ultimate-Goal/blob/master/libOpenCvNative.so**

## Help
For more in depth documentation and configurations, visit these two links:
* [EasyOpenCV](https://github.com/OpenFTC/EasyOpenCV)
* [Javadocs](https://javadoc.io/doc/org.openftc/easyopencv/latest/index.html)

## Contribution
If you would like to contribute to **CobaltCV**, join our discord server!
- https://discord.gg/vm5vHu
