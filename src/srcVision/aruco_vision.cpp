#include "../incVision/vision.h"

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

using namespace aruco;


Mat getRx(double angle)
{
	Mat Rx(3, 3, CV_32FC1);

	Rx.at<float>(0, 0) = 1;
	Rx.at<float>(0, 1) = 0;
	Rx.at<float>(0, 2) = 0;
	Rx.at<float>(1, 0) = 0;
	Rx.at<float>(1, 1) = cos(angle);
	Rx.at<float>(1, 2) = -sin(angle);
	Rx.at<float>(2, 0) = 0;
	Rx.at<float>(2, 1) = sin(angle);
	Rx.at<float>(2, 2) = cos(angle);

	return Rx;
}

Mat getRy(double angle)
{
	Mat Ry(3, 3, CV_32FC1);

	Ry.at<float>(0, 0) = cos(angle);
	Ry.at<float>(0, 1) = 0;
	Ry.at<float>(0, 2) = sin(angle);
	Ry.at<float>(1, 0) = 0;
	Ry.at<float>(1, 1) = 1;
	Ry.at<float>(1, 2) = 0;
	Ry.at<float>(2, 0) = -sin(angle);
	Ry.at<float>(2, 1) = 0;
	Ry.at<float>(2, 2) = cos(angle);

	return Ry;
}

Mat getRz(double angle)
{
	Mat Rz(3, 3, CV_32FC1);

	Rz.at<float>(0, 0) = cos(angle);
	Rz.at<float>(0, 1) = -sin(angle);
	Rz.at<float>(0, 2) = 0;
	Rz.at<float>(1, 0) = sin(angle);
	Rz.at<float>(1, 1) = cos(angle);
	Rz.at<float>(1, 2) = 0;
	Rz.at<float>(2, 0) = 0;
	Rz.at<float>(2, 1) = 0;
	Rz.at<float>(2, 2) = 1;

	return Rz;
}

Mat getRotation(Mat r)
{
	return getRz(r.at<float>(2, 0)) * getRy(r.at<float>(1, 0)) * getRx(r.at<float>(0, 0));
}

int getJointPositions(Mat imgOrg, Arm * arm)
{
	// reset found flag from all joints
	arm->resetJ1Found();
	arm->resetJ2Found();
	arm->resetJ3Found();

  // Create array of camera parameters
	CameraParameters TheCameraParameters;

	// set camera parameters
	Mat dist(1,5,CV_32FC1);
	dist.at<float>(0,0) = -0.0648763971625288;
	dist.at<float>(0,1) = 0.0612520196884308;
	dist.at<float>(0,2) = 0.0038281538281731;
	dist.at<float>(0,3) = -0.00551104078371959;
	dist.at<float>(0,4) = 0.000000;

	Mat cameraP(3,3,CV_32FC1);
	cameraP.at<float>(0,0) = 558.570339530768;
	cameraP.at<float>(0,1) = 0.000000;
	cameraP.at<float>(0,2) = 308.885375457296;
	cameraP.at<float>(1,0) = 0.000000;
	cameraP.at<float>(1,1) = 556.122943034837;
	cameraP.at<float>(1,2) = 247.600724811385;
	cameraP.at<float>(2,0) = 0.000000;
	cameraP.at<float>(2,1) = 0.000000;
	cameraP.at<float>(2,2) = 1.000000;

	TheCameraParameters.setParams(cameraP, dist, CAMERA_RESOLUTION);
	TheCameraParameters.resize(CAMERA_RESOLUTION);

  Vector3d j1, j2, j3;

  // Find Marker and show 3D position
	MarkerDetector MDetector;
  vector<Marker> Markers;

  Mat imgGray = imgOrg.clone();
  cvtColor(imgGray, imgGray, CV_BGR2GRAY);

  MDetector.detect(imgGray, Markers, TheCameraParameters);
  for(int i = 0; i < Markers.size(); i++)
  {
		Markers[i].calculateExtrinsics(0.09, TheCameraParameters);
		Markers[i].draw(imgOrg, Scalar(0,0,255), 2);

		// get abs position in picture for every joint
		if(Markers[i].id == SHOULDER_LEFT)
		{
			// switch x and y axis
			j1(0) = Markers[i].Tvec.at<float>(1, 0);
			j1(1) = Markers[i].Tvec.at<float>(0, 0);
			// invert z-axis
			j1(2) = Markers[i].Tvec.at<float>(2, 0);

			// winkel
			cout << "winkel " << Markers[i].Rvec << endl;


			arm->setJ1Found();

			if(COUT_JOINT_ABS_POS == ON)
			{
				if(arm->getArmName() == "arm_left")
					cout << "abs pos SHOULDER_LEFT: " << j1(0) << "\t" << j1(1) << "\t" << j1(2) << endl;
				else
					cout << "abs pos SHOULDER_RIGHT: " << j1(0) << "\t" << j1(1) << "\t" << j1(2) << endl;
			}
		}
		else if(Markers[i].id == ELBOW_LEFT)
		{
			// switch x and y axis
			j2(0) = Markers[i].Tvec.at<float>(1, 0);
			j2(1) = Markers[i].Tvec.at<float>(0, 0);
			// invert z-axis
			j2(2) = Markers[i].Tvec.at<float>(2, 0);

			arm->setJ2Found();


			if(COUT_JOINT_ABS_POS == ON)
			{
				if(arm->getArmName() == "arm_left")
					cout << "abs pos ELBOW_LEFT: " << j2(0) << "\t" << j2(1) << "\t" << j2(2) << endl;
				else
					cout << "abs pos ELBOW_RIGHT: " << j2(0) << "\t" << j2(1) << "\t" << j2(2) << endl;
			}
		}
		else if(Markers[i].id == WRIST_LEFT)
		{
			// switch x and y axis
			j3(0) = Markers[i].Tvec.at<float>(1, 0);
			j3(1) = Markers[i].Tvec.at<float>(0, 0);
			// invert z-axis
			j3(2) = Markers[i].Tvec.at<float>(2, 0);

			arm->setJ3Found();

			cout << "winkel " << Markers[i].Rvec << endl;


			Mat Rot;
			Rot = getRotation(Markers[i].Rvec);

/*			Mat T(4, 4, CV_32FC1);
			for(int fi = 0; fi < 3; fi++)
			{
				for(int fj = 0; fj < 3; fj++)
					T.at<float>(fi, fj) = Rot.at<float>(fi, fj);

				T.at<float>(fi, 3) = Markers[i].Tvec.at<float>(fi, 0);
			}
			T.at<float>(3, 0) = 0;
			T.at<float>(3, 1) = 0;
			T.at<float>(3, 2) = 0;
			T.at<float>(3, 3) = 1;

			cout << T.at<float>(0, 0) << "\t" << T.at<float>(0, 1) << "\t" << T.at<float>(0, 2) << "\t" << T.at<float>(0, 3) << endl;
			cout << T.at<float>(1, 0) << "\t" << T.at<float>(1, 1) << "\t" << T.at<float>(1, 2) << "\t" << T.at<float>(1, 3) << endl;
			cout << T.at<float>(2, 0) << "\t" << T.at<float>(2, 1) << "\t" << T.at<float>(2, 2) << "\t" << T.at<float>(2, 3) << endl;
			cout << T.at<float>(3, 0) << "\t" << T.at<float>(3, 1) << "\t" << T.at<float>(3, 2) << "\t" << T.at<float>(3, 3) << endl;

			// gauss jordan
*/
			if(COUT_JOINT_ABS_POS == ON)
			{
				if(arm->getArmName() == "arm_left")
					cout << "abs pos WRIST_LEFT: " << j3(0) << "\t" << j3(1) << "\t" << j3(2) << endl;
				else
					cout << "abs pos WRIST_RIGHT: " << j3(0) << "\t" << j3(1) << "\t" << j3(2) << endl;
			}
		}
  }

	// draw circle
	circle(imgOrg, Point(320, 240), 1, Scalar(0, 0, 255));
	circle(imgOrg, Point(320, 240), 100, Scalar(0, 0, 255));

	// Display camera imape with detected marker
	if(SHOW_ARUCO_FOUND_IMG == ON)
  	cv::imshow("arruco", imgOrg);


	// calculate relative position
	if(arm->getJ1Found() == true && arm->getJ2Found() == true && arm->getJ3Found() == true)
	{
		Vector3d d1 = j2 - j1;
		Vector3d d2 = j3 - j1;

		// check min dist
		for(int i = 0; i < 3; i++)
		{
			if(abs(d1(i)) < MIN_DIST)
				d1(i) = 0.0;

			if(abs(d2(i)) < MIN_DIST)
				d2(i) = 0.0;
		}

		cout << "d1 " << d1(0) << "\t" << d1(1) << "\t" << d1(2) << endl;
		cout << "d2 " << d2(0) << "\t" << d2(1) << "\t" << d2(2) << endl;

		// set relative coordinates
		j1(0) = 0;
		j1(1) = 0;
		j1(2) = 0;

		j2(0) = d1(2);
		j2(1) = -d1(1);
		j2(2) = d1(0);

		j3(0) = d2(2);
		j3(1) = -d2(1);
		j3(2) = d2(0);

		// write into arm
		arm->setJ1Coord(j1);
		arm->setJ1Coord(j2);
		arm->setJ1Coord(j3);

		if(COUT_JOINT_REL_POS == ON)
		{
			if(arm->getArmName() == "arm_left")
			{
				cout << "rel pos SHOULDER_LEFT: " 	<< j1(0) << "\t\t" 	<< j1(1) << "\t\t" << j1(2) << endl;
				cout << "rel pos ELBOW_LEFT:    " 	<< j2(0) << "\t" 		<< j2(1) << "\t" << j2(2) << endl;
				cout << "rel pos WRIST_LEFT:    " 	<< j3(0) << "\t" 		<< j3(1) << "\t" << j3(2) << endl;
			}
			else
			{
				cout << "rel pos SHOULDER_RIGHT: " 	<< j1(0) << "\t\t" 	<< j1(1) << "\t\t" << j1(2) << endl;
				cout << "rel pos ELBOW_RIGHT:    "	<< j2(0) << "\t" 		<< j2(1) << "\t" << j2(2) << endl;
				cout << "rel pos WRIST_RIGHT:    " 	<< j3(0) << "\t" 		<< j3(1) << "\t" << j3(2) << endl;
			}

			cout << endl << endl;
		}

		return SUCCESS;
	}
	else
		return FAILURE;
}
