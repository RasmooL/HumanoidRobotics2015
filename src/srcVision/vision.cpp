#include "../incVision/vision.h"


using namespace cv;
using namespace std;
using namespace Eigen;


int getNextState(int currentS)
{
	int nextS;

	switch(currentS)
	{
	case INIT_COLOR_LEFT:
		nextS = DESTROY_WINDOW_INIT_COLOR_LEFT;
		break;

	case INIT_COLOR_RIGHT:
		nextS = DESTROY_WINDOW_INIT_COLOR_RIGHT;
		break;
	
	default:
		nextS = DEFAULT_STATE;
	}
	
	return nextS;
}


void stateInitColor(Mat imgOrg, Arm arm)
{
    // setting up colors
    if(arm.getArmName() == "arm_left")
    {
        namedWindow("control_left", CV_WINDOW_AUTOSIZE);

        // red
        cvCreateTrackbar("iRHLow",  "control_left", &iRHLow, 179); //Hue (0 - 179)
        cvCreateTrackbar("iRHHigh", "control_left", &iRHHigh, 179);
        cvCreateTrackbar("iRSLow",  "control_left", &iRSLow, 255); //Saturation (0 - 255)
        cvCreateTrackbar("iRSHigh", "control_left", &iRSHigh, 255);
        cvCreateTrackbar("iRVLow",  "control_left", &iRVLow, 255); //Value (0 - 255)
        cvCreateTrackbar("iRVHigh", "control_left", &iRVHigh, 255);

        // yellow
        cvCreateTrackbar("iYHLow",  "control_left", &iYHLow, 179); //Hue (0 - 179)
        cvCreateTrackbar("iYHHigh", "control_left", &iYHHigh, 179);
        cvCreateTrackbar("iYSLow",  "control_left", &iYSLow, 255); //Saturation (0 - 255)
        cvCreateTrackbar("iYSHigh", "control_left", &iYSHigh, 255);
        cvCreateTrackbar("iYVLow",  "control_left", &iYVLow, 255); //Value (0 - 255)
        cvCreateTrackbar("iYVHigh", "control_left", &iYVHigh, 255);

        // blue
        cvCreateTrackbar("iBHLow",  "control_left", &iBHLow, 179); //Hue (0 - 179)
        cvCreateTrackbar("iBHHigh", "control_left", &iBHHigh, 179);
        cvCreateTrackbar("iBSLow",  "control_left", &iBSLow, 255); //Saturation (0 - 255)
        cvCreateTrackbar("iBSHigh", "control_left", &iBSHigh, 255);
        cvCreateTrackbar("iBVLow",  "control_left", &iBVLow, 255); //Value (0 - 255)
        cvCreateTrackbar("iBVHigh", "control_left", &iBVHigh, 255);

    }
    else if(arm.getArmName() == "arm_right")
    {
        namedWindow("control_right", CV_WINDOW_AUTOSIZE);

        // green
        cvCreateTrackbar("iGHLow",  "control_right", &iGHLow, 179); //Hue (0 - 179)
        cvCreateTrackbar("iGHHigh", "control_right", &iGHHigh, 179);
        cvCreateTrackbar("iGSLow",  "control_right", &iGSLow, 255); //Saturation (0 - 255)
        cvCreateTrackbar("iGSHigh", "control_right", &iGSHigh, 255);
        cvCreateTrackbar("iGVLow",  "control_right", &iGVLow, 255); //Value (0 - 255)
        cvCreateTrackbar("iGVHigh", "control_right", &iGVHigh, 255);

        // dark_blue
        cvCreateTrackbar("iDBHLow",  "control_right", &iDBHLow, 179); //Hue (0 - 179)
        cvCreateTrackbar("iDBHHigh", "control_right", &iDBHHigh, 179);
        cvCreateTrackbar("iDBSLow",  "control_right", &iDBSLow, 255); //Saturation (0 - 255)
        cvCreateTrackbar("iDBSHigh", "control_right", &iDBSHigh, 255);
        cvCreateTrackbar("iDBVLow",  "control_right", &iDBVLow, 255); //Value (0 - 255)
        cvCreateTrackbar("iDBVHigh", "control_right", &iDBVHigh, 255);

        // brown
        cvCreateTrackbar("iBRHLow",  "control_right", &iBRHLow, 179); //Hue (0 - 179)
        cvCreateTrackbar("iBRHHigh", "control_right", &iBRHHigh, 179);
        cvCreateTrackbar("iBRSLow",  "control_right", &iBRSLow, 255); //Saturation (0 - 255)
        cvCreateTrackbar("iBRSHigh", "control_right", &iBRSHigh, 255);
        cvCreateTrackbar("iBRVLow",  "control_right", &iBRVLow, 255); //Value (0 - 255)
        cvCreateTrackbar("iBRVHigh", "control_right", &iBRVHigh, 255);
    }

    Mat imgHSV;

    cvtColor(imgOrg, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    // update color values
    if(arm.getArmName() == "arm_left")
    {
        sRedLow         = Scalar(iRHLow, iRSLow, iRVLow);
        sRedHigh        = Scalar(iRHHigh, iRSHigh, iRVHigh);
        sYellowLow      = Scalar(iYHLow, iYSLow, iYVLow);
        sYellowHigh     = Scalar(iYHHigh, iYSHigh, iYVHigh);
        sBlueLow        = Scalar(iBHLow, iBSLow, iBVLow);
        sBlueHigh       = Scalar(iBHHigh, iBSHigh, iBVHigh);
    }
    else if(arm.getArmName() == "arm_right")
    {
        sGreenLow       = Scalar(iGHLow, iGSLow, iGVLow);
        sGreenHigh      = Scalar(iGHHigh, iGSHigh, iGVHigh);
        sDarkBlueLow    = Scalar(iDBHLow, iDBSLow, iDBVLow);
        sDarkBlueHigh   = Scalar(iDBHHigh, iDBSHigh, iDBVHigh);
        sBrownLow       = Scalar(iBRHLow, iBRSLow, iBRVLow);
        sBrownHigh      = Scalar(iBRHHigh, iBRSHigh, iBRVHigh);
    }


    Mat imgThreshold;
	for(int i = 0; i < 3; i++)
	{
		// get HSV
		if (arm.getArmName() == "arm_left")
		{
			if(i == 0)
				inRange(imgHSV, sRedLow, sRedHigh, imgThreshold);
			else if(i == 1)
				inRange(imgHSV, sYellowLow, sYellowHigh, imgThreshold);
			else if(i == 2)
				inRange(imgHSV, sBlueLow, sBlueHigh, imgThreshold);
		}
		
		else if(arm.getArmName() == "arm_right")
		{
			if(i == 0)
				inRange(imgHSV, sGreenLow, sGreenHigh, imgThreshold);
			else if(i == 1)
				inRange(imgHSV, sDarkBlueLow, sDarkBlueHigh, imgThreshold);
			else if(i == 2)
				inRange(imgHSV, sBrownLow, sBrownHigh, imgThreshold);
		}



		//morphological opening (remove small objects from the foreground)
		erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		// show each color without background
		if(SHOW_WITHOUT_BG_IMG == ON)
		{
//			if(arm.getArmName() == "arm_left")
//			{
				if(i == 0)
				{
					Mat imgRed = Mat::zeros(imgOrg.size(), imgOrg.type());
					for(int y = 0; y < imgOrg.rows; y++)
					{
		    			for(int x = 0; x < imgOrg.cols; x++)
		    			{
		        			if(imgThreshold.at<uchar>(y, x) == MAX_THRESH_VAL)
		            			imgRed.at<Vec3b>(Point(x, y)) = imgOrg.at<Vec3b>(Point(x, y));
		    			}
					}
					imshow("imgRed", imgRed);
				}

				else if(i == 1)
				{
					Mat imgYellow = Mat::zeros(imgOrg.size(), imgOrg.type());
					for(int y = 0; y < imgOrg.rows; y++)
					{
		    			for(int x = 0; x < imgOrg.cols; x++)
		    			{
		        			if(imgThreshold.at<uchar>(y, x) == MAX_THRESH_VAL)
		            			imgYellow.at<Vec3b>(Point(x, y)) = imgOrg.at<Vec3b>(Point(x, y));
		    			}
					}
					imshow("imgYellow", imgYellow);
				}

				else if(i == 2)
				{
					Mat imgBlue = Mat::zeros(imgOrg.size(), imgOrg.type());
					for(int y = 0; y < imgOrg.rows; y++)
					{
		    			for(int x = 0; x < imgOrg.cols; x++)
		    			{
		        			if(imgThreshold.at<uchar>(y, x) == MAX_THRESH_VAL)
		            			imgBlue.at<Vec3b>(Point(x, y)) = imgOrg.at<Vec3b>(Point(x, y));
		    			}
					}
					imshow("imgBlue", imgBlue);
				}
//			}
			
/*			else if(arm.getArmName() == "arm_right")
			{
				if(i == 0)
				{
					Mat imgGreen = Mat::zeros(imgOrg.size(), imgOrg.type());
					for(int y = 0; y < imgOrg.rows; y++)
					{
		    			for(int x = 0; x < imgOrg.cols; x++)
		    			{
		        			if(imgThreshold.at<uchar>(y, x) == MAX_THRESH_VAL)
		            			imgGreen.at<Vec3b>(Point(x, y)) = imgOrg.at<Vec3b>(Point(x, y));
		    			}
					}
					imshow("imgGreen", imgGreen);
				}
				else if(i == 1)
				{
					Mat imgDarkBlue = Mat::zeros(imgOrg.size(), imgOrg.type());
					for(int y = 0; y < imgOrg.rows; y++)
					{
		    			for(int x = 0; x < imgOrg.cols; x++)
		    			{
		        			if(imgThreshold.at<uchar>(y, x) == MAX_THRESH_VAL)
		            			imgDarkBlue.at<Vec3b>(Point(x, y)) = imgOrg.at<Vec3b>(Point(x, y));
		    			}
					}
					imshow("imgDarkBlue", imgDarkBlue);
				}
				else if(i == 2)
				{
					Mat imgBrown = Mat::zeros(imgOrg.size(), imgOrg.type());
					for(int y = 0; y < imgOrg.rows; y++)
					{
		    			for(int x = 0; x < imgOrg.cols; x++)
		    			{
		        			if(imgThreshold.at<uchar>(y, x) == MAX_THRESH_VAL)
		            			imgBrown.at<Vec3b>(Point(x, y)) = imgOrg.at<Vec3b>(Point(x, y));
		    			}
					}
					imshow("imgBrown", imgBrown);
				}
			}
*/
		}
	}
}



int stateInitArm(Mat imgOrg, Arm *arm)
{
	int nextState;
	// set nextState to stay in function
	if(arm->getArmName() == "arm_left")
		nextState = INIT_ARM_RIGHT;
	else if(arm->getArmName() == "arm_right")
		nextState = RUN_BODY_TRACKING;

    // convert into HSV
    Mat imgHSV;
    cvtColor(imgOrg, imgHSV, COLOR_BGR2HSV);

    Mat withoutBG = Mat::zeros(imgOrg.size(), imgOrg.type());

    // set centers of points
    arm->setJ1Center(getJCenter(imgHSV.clone(), imgOrg, withoutBG, arm->getJ1Color()));
    arm->setJ2Center(getJCenter(imgHSV.clone(), imgOrg, withoutBG, arm->getJ2Color()));
    arm->setJ3Center(getJCenter(imgHSV.clone(), imgOrg, withoutBG, arm->getJ3Color()));

    // get center point of each joint
    Point p1 = arm->getJ1Center();
    Point p2 = arm->getJ2Center();
    Point p3 = arm->getJ3Center();

    // distance between p1 and p2
    int dx1 = p1.x - p2.x;
    int dy1 = p1.y - p2.y;

    // distance between p2 and p3
    int dx2 = p2.x - p3.x;
    int dy2 = p2.y - p3.y;

    // set bone length
    arm->setBone1(sqrt(dx1 * dx1 + dy1 * dy1));
    arm->setBone2(sqrt(dx2 * dx2 + dy2 * dy2));

    if(arm->getArmName() == "arm_left")
        cout << "initialization of left arm finished\n";
    else if(arm->getArmName() == "arm_right")
        cout << "initialization of right arm finished\n";

    return nextState;
}



void drawModel(Mat imgOrg, Arm *arm)
{
    // all 3 joints
    if(arm->getJ1Center().x != 0 && arm->getJ1Center().y != 0 &&
       arm->getJ2Center().x != 0 && arm->getJ2Center().y != 0 &&
       arm->getJ3Center().x != 0 && arm->getJ3Center().y != 0)
    {
        // draw joints
        circle(imgOrg, arm->getJ1Center(), jointRadius, jointColor);
        circle(imgOrg, arm->getJ2Center(), jointRadius, jointColor);
        circle(imgOrg, arm->getJ3Center(), jointRadius, jointColor);

        // draw lines between joints
        line(imgOrg, arm->getJ1Center(), arm->getJ2Center(), lineColor);
        line(imgOrg, arm->getJ2Center(), arm->getJ3Center(), lineColor);

        imshow("orig & model", imgOrg);
    }

    // not enought joints jound
    else
    {
        // print info
        cout << "not enough joints found in " << arm->getArmName() << endl;
        cout << "j1 " << arm->getJ1Center() << endl;
        cout << "j2 " << arm->getJ2Center() << endl;
        cout << "j3 " << arm->getJ3Center() << endl;
    }
}



int calculatePosition(Arm *arm)
{
    Point p1 = arm->getJ1Center();
    Point p2 = arm->getJ2Center();
    Point p3 = arm->getJ3Center();

    // check if all joints are detected
    if((p1.x != 0 || p1.y != 0) && (p2.x != 0 || p2.y != 0) && (p3.x != 0 || p2.y != 0))
    {
        int bone1 = arm->getBone1();
        int bone2 = arm->getBone2();

//        cout << arm->getArmName() << endl;
//        cout << p1 << " " << p2 << " " << p3 << endl;


        // LINK 1
        // calc relative joint position from shoulder to elbow
        int dx = p2.x - p1.x;
        int dz = p2.y - p1.y;

        // calc distance from shoulder to elbow
        double link1 = sqrt(dx * dx + dz * dz);


        // calc angle between shoulder and elbow
        double alpha = atan((double)dx / dz);
        //check if alpha is nan (very close to 0)
        if(alpha != alpha)
            alpha = 0;

        // calc depth angle
        double beta = acos((double)link1 / bone1);
        // check if beta is nan (very close to 0)
        if(beta != beta)
            beta = 0;

        // translate link1 from human to NAO
        double normalizedLink1 = link1 / bone1 * BONE1ROBOT;

        // calc NAO coord
        Vector3d coordNAO1;
//        coordNAO1(0) = link1NAO * sin(alpha1);
//        coordNAO1(1) = link1NAO * sin(alpha2);
//        coordNAO1(2) = - link1NAO * cos(alpha1);
        coordNAO1(0) = normalizedLink1 * sin(beta);
        coordNAO1(1) = normalizedLink1 * sin(alpha);
        coordNAO1(2) = - normalizedLink1 * cos(alpha);

//        cout << "coord1 " << coordNAO1(0) << " " << coordNAO1(1) << " " << coordNAO1(2) << endl;



        // LINK 2
//        // calc relative joint position from elbow to wrist
        dx = p3.x - p2.x;
        dz = p3.y - p2.y;

        // calc distance from elbow to wrist
        double link2 = sqrt(dx * dx + dz * dz);

        // calc angle between shoulder and elbow
        alpha = atan((double)dx / dz);
        //check if alpha is nan (very close to 0)
        if(alpha != alpha)
            alpha = 0;

        // calc depth angle
        beta = acos((double)link2 / bone2);
        // check if beta is nan (very close to 0)
        if(beta != beta)
            beta = 0;

        // translate link1 from human to NAO
        double normalizedLink2 = link2 / bone2 * BONE2ROBOT;


        // calc NAO coord
        Vector3d coordNAO2;
//        coordNAO2(0) = link2NAO * sin(alpha1) + coordNAO1(0);
//        coordNAO2(1) = link2NAO * sin(alpha2) + coordNAO1(1);
//        coordNAO2(2) = - link2NAO * cos(alpha1) + coordNAO1(2);
        coordNAO2(0) = normalizedLink2 * sin(beta) + coordNAO1(0);
        coordNAO2(1) = normalizedLink2 * sin(alpha) + coordNAO1(1);
        coordNAO2(2) = - normalizedLink2 * cos(alpha) + coordNAO1(2);

//        cout << "coord2 " << coordNAO2(0) << " " << coordNAO2(1) << " " << coordNAO2(2) << endl;

        arm->setJ2Coord(coordNAO1);
        arm->setJ3Coord(coordNAO2);
    }
    else
        return FAILURE;

    return SUCCESS;
}



int runArmTracking(Mat imgOrg, Arm *arm)
{
	if(SHOW_ORIGINAL_IMG == ON)
        imshow("imgOrg", imgOrg);

    // convert into HSV
    Mat imgHSV;
    cvtColor(imgOrg, imgHSV, COLOR_BGR2HSV);

    Mat withoutBG = Mat::zeros(imgOrg.size(), imgOrg.type());

    arm->setJ1Center(getJCenter(imgHSV.clone(), imgOrg, withoutBG, arm->getJ1Color()));
    arm->setJ2Center(getJCenter(imgHSV.clone(), imgOrg, withoutBG, arm->getJ2Color()));
    arm->setJ3Center(getJCenter(imgHSV.clone(), imgOrg, withoutBG, arm->getJ3Color()));

    if(SHOW_WITHOUT_BG_IMG == ON)
        imshow("withoutBG", withoutBG);

    // draw model into image
    drawModel(imgOrg, arm);

    // calculate position
	if(calculatePosition(arm) == SUCCESS)
		return SUCCESS;
	else
		return FAILURE;

/*    if(calculatePosition(&arm_left) == SUCCESS)
    {
        cout << arm_left->getArmName() << endl;

        Vector3d jointLeft;

        jointLeft = arm_left->getJ1Coord();
        cout << "j1   " << jointLeft(0) << "  " << jointLeft(1) << "  " << jointLeft(2) << endl;

        jointLeft = arm_left->getJ2Coord();
        cout << "j2   " << jointLeft(0) << "  " << jointLeft(1) << "  " << jointLeft(2) << endl;

        jointLeft = arm_left.getJ3Coord();
        cout << "j3   " << jointLeft(0) << "  " << jointLeft(1) << "  " << jointLeft(2) << endl;
    }
    else
        cout << "not all joints for left arm found!\n";


    if(calculatePosition(&arm_right) == SUCCESS)
    {
        cout << arm_right->getArmName() << endl;

        Vector3d jointRight;

        jointRight = arm_right->getJ1Coord();
        cout << "j1   " << jointRight(0) << "  " << jointRight(1) << "  " << jointRight(2) << endl;

        jointRight = arm_right->getJ2Coord();
        cout << "j2   " << jointRight(0) << "  " << jointRight(1) << "  " << jointRight(2) << endl;

        jointRight = arm_right->getJ3Coord();
        cout << "j3   " << jointRight(0) << "  " << jointRight(1) << "  " << jointRight(2) << endl << endl << endl;
    }
    else
        cout << "not all joints for right arm found\n";
*/

}

