//Gate detection and tracking

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int main()
{
    // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    KalmanFilter kf(stateSize, measSize, contrSize, type);

    Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
     Mat procNoise(stateSize, 1, type);
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    
    //setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    setIdentity(kf.processNoiseCov, Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    
    setIdentity(kf.measurementNoiseCov, Scalar(1e-1));
    


    VideoCapture cap("3.avi");
    while(1)
    {

 
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 360);

    Mat frame;
    cap >> frame;
    
    if(!cap.isOpened())
    {
        cout << "Error" << endl;
        return -1;
    }
    
    cout << "Press 'q' or 'Q' to exit\n";

    
    bool found = false;

    int notFoundCount = 0;

    
        Mat frame2;
        frame.copyTo(frame2);

        if (found)
        {
            state = kf.predict();
            cout << "State post:" << endl << state << endl;

            Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            //Point center;
            //center.x = state.at<float>(0);
            //center.y = state.at<float>(1);
            //circle(frame2, center, 2, CV_RGB(255,0,0), -1);

            rectangle(frame2, predRect, CV_RGB(255,0,0), 2);
        }

        
        Mat blur;
        GaussianBlur(frame, blur, Size(5, 5), 5.0, 5.0);
        

        
        //Mat frmHsv;
        //cvtColor(blur, frmHsv, CV_BGR2HSV);
        

        
        
        Mat rangeRes = Mat::zeros(frame.size(), CV_8UC1);
        inRange(frame, Scalar(0, 0, 0),Scalar(39, 145, 109), rangeRes);
        

        
        //erode(rangeRes, rangeRes, Mat(), Point(-1, -1), 2);
        //dilate(rangeRes, rangeRes, Mat(), Point(-1, -1), 2);
        

        
        imshow("Thresholding", rangeRes);

        // Contours detection
        vector<vector<Point> > contours;
        findContours(rangeRes, contours, CV_RETR_EXTERNAL,
                         CV_CHAIN_APPROX_NONE);
        // Contours detection

        // Filtering
        vector<vector<Point> > gate;
        vector<Rect> gateBox;
        for (size_t i = 0; i < contours.size(); i++)
        {
            Rect bBox;
            bBox = boundingRect(contours[i]);

            float ratio = (float) bBox.width / (float) bBox.height;
            if (ratio > 1.0f)
                ratio = 1.0f / ratio;

            // Searching for a bBox almost square
            if (ratio > 0.75 && bBox.area() >= 400)
            {
                gate.push_back(contours[i]);
                gateBox.push_back(bBox);
            }
        }
        
        for (size_t i = 0; i < gate.size(); i++)
        {
            drawContours(frame2, gate, i, CV_RGB(20,150,20), 1);
            rectangle(frame2, gateBox[i], CV_RGB(0,255,0), 2);

            /*Point center;
            center.x = gateBox[i].x + gateBox[i].width / 2;
            center.y = gateBox[i].y + gateBox[i].height / 2;
            circle(frame2, center, 2, CV_RGB(20,150,20), -1);

            stringstream coord;
            coord << "(" << center.x << "," << center.y << ")";
            putText(frame2, coord.str(),Point(center.x + 3, center.y - 3),FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(50,150,50), 2);
            */
        }
        

        // Kalman Update
        if (gate.size() == 0)
        {
            notFoundCount++;
            cout << "notFoundCount:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
            }
            else
            kf.statePost = state;
        }
        else
        {
            notFoundCount = 0;

            meas.at<float>(0) = gateBox[0].x + gateBox[0].width / 2;
            meas.at<float>(1) = gateBox[0].y + gateBox[0].height / 2;
            meas.at<float>(2) = (float)gateBox[0].width;
            meas.at<float>(3) = (float)gateBox[0].height;    

            if (!found) 
            {
                
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px
                kf.errorCovPre.at<float>(35) = 1; // px

                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                state.at<float>(4) = meas.at<float>(2);
                state.at<float>(5) = meas.at<float>(3);
                

                kf.statePost = state;
                
                found = true;
            }
            else
                kf.correct(meas); // Kalman Correction

            //cout << "Measure matrix:" << endl << meas << endl;
        }
        // Kalman Update

      


        imshow("Gate Detection", frame2);
        imshow("frame",frame);
        waitKey(1);
}

cap.release();
destroyAllWindows();

 waitKey(25);


    return EXIT_SUCCESS;
}
