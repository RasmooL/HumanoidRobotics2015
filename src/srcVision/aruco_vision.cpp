// Zylka, Adam

#include "../incVision/vision.h"

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

using namespace aruco;


void getJointPositions(Mat imgOrg, Arm *arm_left, Arm *arm_right, Chest *chest)
{
	// reset found flag from all joints and both arms
	arm_left->resetJ1Found();
	arm_left->resetJ2Found();
	arm_left->resetJ3Found();
	arm_left->resetArmFound();

	arm_right->resetJ1Found();
	arm_right->resetJ2Found();
	arm_right->resetJ3Found();
	arm_right->resetArmFound();

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

	// create vectors for joints
  Vector3d j1Left, j2Left, j3Left;
	Vector3d j1Right, j2Right, j3Right;

	// create objects for marker
	MarkerDetector MDetector;
  vector<Marker> Markers;

	// set marker settings
	MDetector.setWarpSize(100);
	MDetector.enableLockedCornersMethod(true);
	MDetector.setMinMaxSize(0.01, 0.5);

	// convert image to gray
  Mat imgGray = imgOrg.clone();
  cvtColor(imgGray, imgGray, CV_BGR2GRAY);

	// detect all markers in the picture
  MDetector.detect(imgGray, Markers, TheCameraParameters);

	// go through all found markers
  for(int i = 0; i < Markers.size(); i++)
  {
			Markers[i].calculateExtrinsics(0.09, TheCameraParameters);
			Markers[i].draw(imgOrg, Scalar(0,0,255), 2);

			// get abs position in picture for every joint
			switch(Markers[i].id)
			{
			case SHOULDER_LEFT:
					// switch x and y axis
					j1Left(0) = Markers[i].Tvec.at<float>(1, 0);
					j1Left(1) = Markers[i].Tvec.at<float>(0, 0);
					j1Left(2) = Markers[i].Tvec.at<float>(2, 0);

					arm_left->setJ1Found();

					if(COUT_JOINT_ABS_POS == ON)
					{
							cout << "abs pos SHOULDER_LEFT: " << j1Left(0) << "\t" << j1Left(1) << "\t" << j1Left(2) << endl;
					}
					break;

			case ELBOW_LEFT:
					// switch x and y axis
					j2Left(0) = Markers[i].Tvec.at<float>(1, 0);
					j2Left(1) = Markers[i].Tvec.at<float>(0, 0);
					j2Left(2) = Markers[i].Tvec.at<float>(2, 0);

					arm_left->setJ2Found();

					if(COUT_JOINT_ABS_POS == ON)
					{
							cout << "abs pos ELBOW_LEFT: " << j2Left(0) << "\t" << j2Left(1) << "\t" << j2Left(2) << endl;
					}
					break;

			case WRIST_LEFT:
					// switch x and y axis
					j3Left(0) = Markers[i].Tvec.at<float>(1, 0);
					j3Left(1) = Markers[i].Tvec.at<float>(0, 0);
					j3Left(2) = Markers[i].Tvec.at<float>(2, 0);

					arm_left->setJ3Found();

					if(COUT_JOINT_ABS_POS == ON)
					{
							cout << "abs pos WRIST_LEFT: " << j3Left(0) << "\t" << j3Left(1) << "\t" << j3Left(2) << endl;
					}
					break;

				case SHOULDER_RIGHT:
						// switch x and y axis
						j1Right(0) = Markers[i].Tvec.at<float>(1, 0);
						j1Right(1) = Markers[i].Tvec.at<float>(0, 0);
						j1Right(2) = Markers[i].Tvec.at<float>(2, 0);

						arm_right->setJ1Found();

						if(COUT_JOINT_ABS_POS == ON)
						{
								cout << "abs pos SHOULDER_RIGHT: " << j1Right(0) << "\t" << j1Right(1) << "\t" << j1Right(2) << endl;
						}
						break;

				case ELBOW_RIGHT:
						// switch x and y axis
						j2Right(0) = Markers[i].Tvec.at<float>(1, 0);
						j2Right(1) = Markers[i].Tvec.at<float>(0, 0);
						j2Right(2) = Markers[i].Tvec.at<float>(2, 0);

						arm_right->setJ2Found();

						if(COUT_JOINT_ABS_POS == ON)
						{
								cout << "abs pos ELBOW_RIGHT: " << j2Right(0) << "\t" << j2Right(1) << "\t" << j2Right(2) << endl;
						}
						break;

				case WRIST_RIGHT:
						// switch x and y axis
						j3Right(0) = Markers[i].Tvec.at<float>(1, 0);
						j3Right(1) = Markers[i].Tvec.at<float>(0, 0);
						j3Right(2) = Markers[i].Tvec.at<float>(2, 0);

						arm_right->setJ3Found();

						if(COUT_JOINT_ABS_POS == ON)
						{
								cout << "abs pos WRIST_RIGHT: " << j3Right(0) << "\t" << j3Right(1) << "\t" << j3Right(2) << endl;
						}
						break;
				}
		}

		// Display camera imape with detected marker
		if(SHOW_ARUCO_FOUND_IMG == ON)
	  		cv::imshow("arruco", imgOrg);


		// set initial position for upper boddy
		if((chest->getInit() == false) && arm_left->getJ1Found() && arm_right->getJ1Found())
		{
				// set bool to true
				chest->setInit();

				// set initial position between shoulders -> both shoulders are needed
				if(arm_left->getJ1Found() && arm_right->getJ1Found())
					chest->setInitPos((-j1Left(2) -j1Right(2))/2);

				if(COUT_CHEST_INIT_POS == ON)
						cout << "chest init pos: " << chest->getInitPos() << endl;
		}


		// check if upper boddy moved
		if(chest->getInit() && (arm_left->getJ1Found() || arm_right->getJ1Found()))
		{
				// reset bool
				chest->resetTorsoMoved();

				// calculate current chest position
				double chestCurPos;
				if(arm_left->getJ1Found() && arm_right->getJ1Found())
						chestCurPos = (-j1Left(2) -j1Right(2))/2;
				else if(arm_left->getJ1Found())
						chestCurPos = -j1Left(2);
				else if(arm_right->getJ1Found())
						chestCurPos = -j1Right(2);

				if(COUT_CHEST_CUR_POS == ON)
						cout << "current pos: "	<< chestCurPos << endl;


				// calculate distance
				double dist = chest->getInitPos() - chestCurPos;

				if(COUT_CHEST_DIST == ON)
					cout << "chest dist: " << dist << endl;

				// check if distance is greater than threshold
				if(abs(dist) > CHEST_DIST_THRESH)
				{
						// set bool: thresh moved
						chest->setTorsoMoved();

						// calculate angle for torso
						chest->setAngle(asin(dist/TORSO_LENGTH));

						if(COUT_CHEST_ANGLE == ON)
							cout << "angle" << chest->getAngle() << endl;
				}
		}


		// calculate relative position for the left arm
		if(arm_left->getJ1Found() && arm_left->getJ2Found() && arm_left->getJ3Found())
		{
				// calculate distance
				Vector3d d1 = j2Left - j1Left;
				Vector3d d2 = j3Left - j1Left;

				// check min dist
				for(int i = 0; i < 3; i++)
				{
						if(abs(d1(i)) < MIN_DIST)
								d1(i) = 0.0;

						if(abs(d2(i)) < MIN_DIST)
								d2(i) = 0.0;
				}

//				cout << "d1 " << d1(0) << "\t" << d1(1) << "\t" << d1(2) << endl;
//				cout << "d2 " << d2(0) << "\t" << d2(1) << "\t" << d2(2) << endl;

				// set relative coordinates
				j1Left(0) = 0;
				j1Left(1) = 0;
				j1Left(2) = 0;

				j2Left(0) = -d1(2);
				j2Left(1) = d1(1);
				j2Left(2) = -d1(0);

				j3Left(0) = -d2(2);
				j3Left(1) = d2(1);
				j3Left(2) = -d2(0);

				// write into arm
				arm_left->setJ1Coord(j1Left);
				arm_left->setJ2Coord(j2Left);
				arm_left->setJ3Coord(j3Left);

				// set bool that all markers for the left arm have been found
				arm_left->setArmFound();

				if(COUT_JOINT_REL_POS == ON)
				{
						cout << "rel pos SHOULDER_LEFT: " 	<< j1Left(0) << "\t\t" 	<< j1Left(1) << "\t\t" 	<< j1Left(2) << endl;
						cout << "rel pos ELBOW_LEFT:    " 	<< j2Left(0) << "\t" 		<< j2Left(1) << "\t" 		<< j2Left(2) << endl;
						cout << "rel pos WRIST_LEFT:    " 	<< j3Left(0) << "\t" 		<< j3Left(1) << "\t" 		<< j3Left(2) << endl;

						cout << endl << endl;
				}
		}


		// calculate relative position for the right arm
		if(arm_right->getJ1Found() && arm_right->getJ2Found() && arm_right->getJ3Found())
		{
				// calculate distance
				Vector3d d1 = j2Right - j1Right;
				Vector3d d2 = j3Right - j1Right;

				// check min dist
				for(int i = 0; i < 3; i++)
				{
						if(abs(d1(i)) < MIN_DIST)
								d1(i) = 0.0;

						if(abs(d2(i)) < MIN_DIST)
								d2(i) = 0.0;
				}

//				cout << "d1 " << d1(0) << "\t" << d1(1) << "\t" << d1(2) << endl;
//				cout << "d2 " << d2(0) << "\t" << d2(1) << "\t" << d2(2) << endl;

				// set relative coordinates
				j1Right(0) = 0;
				j1Right(1) = 0;
				j1Right(2) = 0;

				j2Right(0) = -d1(2);
				j2Right(1) = d1(1);
				j2Right(2) = -d1(0);

				j3Right(0) = -d2(2);
				j3Right(1) = d2(1);
				j3Right(2) = -d2(0);

				// write into arm
				arm_right->setJ1Coord(j1Right);
				arm_right->setJ2Coord(j2Right);
				arm_right->setJ3Coord(j3Right);

				// set bool that all markers for the right arm have been found
				arm_right->setArmFound();

				if(COUT_JOINT_REL_POS == ON)
				{
						cout << "rel pos SHOULDER_RIGHT: " 	<< j1Right(0) << "\t\t" 	<< j1Right(1) << "\t\t" 	<< j1Right(2) << endl;
						cout << "rel pos ELBOW_RIGHT:    " 	<< j2Right(0) << "\t" 		<< j2Right(1) << "\t" 		<< j2Right(2) << endl;
						cout << "rel pos WRIST_RIGHT:    " 	<< j3Right(0) << "\t" 		<< j3Right(1) << "\t" 		<< j3Right(2) << endl;

						cout << endl << endl;
				}
		}
}
