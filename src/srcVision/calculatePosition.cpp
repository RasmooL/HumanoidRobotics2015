#include "../incVision/calculatePosition.h"
#include "../incVision/constants.h"
#include "../incVision/settings.h"
#include "../incVision/joint.h"

Mat getBin(Mat *imgThreshold)
{
    Mat imgBin;
    Canny(*imgThreshold, imgBin, BIN_THRESHOLD, BIN_THRESHOLD*2, 3);

    if(SHOW_BIN_IMG == ON)
        imshow("bin", imgBin);

    return imgBin;
}


vector<vector<Point> > getContours(Mat imgBin)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours
    findContours(imgBin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    //delete too long or too short contours
    vector<vector<Point> >::iterator itc = contours.begin();
    while(itc!= contours.end())
    {
        if( (itc->size() < MIN_CONTOUR_LENGTH) || (itc->size() > MAX_CONTOUR_LENGTH) )
            itc = contours.erase(itc);
        else
            ++itc;
    }

    if(SHOW_CONTOURS == ON)
    {
        //draw contours -> there are more objects with the same or almost the same coordinates and size
        Mat imgContours = Mat::zeros( imgBin.size(), CV_8UC3 );
        for(int i = 0; i < contours.size(); i++)
        {
            drawContours( imgContours, contours, i, contourColor, 1, 8, hierarchy, 0, Point() );
        }
        imshow("imgContours", imgContours);
    }

    return contours;
}

vector<Rect> getRectangles(Mat imgBin, vector<vector<Point> > contours)
{
    //find rectangles
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    for( int i = 0; i < contours.size(); i++ )
    {
        approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
    }

    //delete the same objects in vector only if there are some contours
    if(contours.size() > 0)
    {
        vector<Rect>::iterator itr = boundRect.begin();
        vector<Rect>::iterator itr_tmp = itr;

        //step over the first rectangle
        itr++;

        while(itr!= boundRect.end())
        {
            if( ((itr->x        >= (itr_tmp->x      - SAME_OBJECT_BORDER))  &&  //x-achsis
                 (itr->x         <= (itr_tmp->x      + SAME_OBJECT_BORDER))) &&
                ((itr->y        >= (itr_tmp->y      - SAME_OBJECT_BORDER))  &&  //y-achsis
                 (itr->y         <= (itr_tmp->y      + SAME_OBJECT_BORDER))) &&
                ((itr->width    >= (itr_tmp->width  - SAME_OBJECT_SIZE))    &&  //wide
                 (itr->width     <= (itr_tmp->width  + SAME_OBJECT_SIZE)))   &&
                ((itr->height   >= (itr_tmp->height - SAME_OBJECT_SIZE))    &&  //height
                 (itr->height    <= (itr_tmp->height + SAME_OBJECT_SIZE))))
            {
                itr = boundRect.erase(itr);
            }
            else
            {
                itr_tmp = itr;
                ++itr;
            }
        }
    }

    //draw found rectangles
    if(SHOW_RECTANGLES == ON)
    {
        Mat imgRect = Mat::zeros(imgBin.size(), CV_8UC3);
        for( int i = 0; i < boundRect.size(); i++ )
        {
            rectangle(imgRect, boundRect[i].tl(), boundRect[i].br(), contourColor, 1, 8, 0);
        }
        imshow("imgRect", imgRect);
    }

    return boundRect;
}


Point getJCenter(Mat imgHSV, Mat imgOriginal, Mat withoutBG, String color)
{
    Point centerPoint;

    Mat imgThreshold;

    // distinguish between the colors
    if(color == "red")
        inRange(imgHSV, sRedLow, sRedHigh, imgThreshold);
    else if(color == "yellow")
        inRange(imgHSV, sYellowLow, sYellowHigh, imgThreshold);
    else if(color == "blue")
        inRange(imgHSV, sBlueLow, sBlueHigh, imgThreshold);
    else if(color == "green")
        inRange(imgHSV, sGreenLow, sGreenHigh, imgThreshold);
    else if(color == "dark_blue")
        inRange(imgHSV, sDarkBlueLow, sDarkBlueHigh, imgThreshold);
    else if(color == "brown")
        inRange(imgHSV, sBrownLow, sBrownHigh, imgThreshold);

    //morphological opening (remove small objects from the foreground)
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    //morphological closing (fill small holes in the foreground)
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    if(SHOW_THRESHOLD_IMG == ON)
        imshow("threshold", imgThreshold);

    // Mat without background
    for(int y = 0; y < imgOriginal.rows; y++)
    {
        for(int x = 0; x < imgOriginal.cols; x++)
        {
            if(imgThreshold.at<uchar>(y, x) == MAX_THRESH_VAL)
                withoutBG.at<Vec3b>(Point(x, y)) = imgOriginal.at<Vec3b>(Point(x, y));
        }
    }

    // create bin image out of threschold image
    Mat imgBin = getBin(&imgThreshold);

    // search for contours
    vector<vector<Point> > contours = getContours(imgBin);

    // proceed only if contours are found
    if(contours.size() > 0)
    {
        // create rectangles around contours
        vector<Rect> rectangles = getRectangles(imgBin, contours);
        if(rectangles.size() == 1)
        {
            centerPoint.x = rectangles[0].x + rectangles[0].width/2;
            centerPoint.y = rectangles[0].y + rectangles[0].height/2;
        }
        else
        {
            if(color == "brown")
                cout << "searching error\n";

            for(int i = 0; i < rectangles.size(); i++)
            {
                if(color == "brown")
                    cout << "x = " << rectangles[i].x << " y = " << rectangles[i].y << endl;

                centerPoint.x += rectangles[i].x + rectangles[i].width/2;
                centerPoint.y += rectangles[i].y + rectangles[i].height/2;
            }
            centerPoint.x = centerPoint.x / rectangles.size();
            centerPoint.y = centerPoint.y / rectangles.size();

            if(color == "brown")
                cout << centerPoint << endl;
        }
    }

    return centerPoint;
}
