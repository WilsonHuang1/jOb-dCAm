/**
 * RGB-D-Inertial TUM dataset format
 * Combines RGB-D processing with IMU data
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, 
             vector<cv::Point3f> &vGyro);

int main(int argc, char **argv)
{
    cout << "[DEBUG] Starting program with " << argc << " arguments" << endl;
    
    if(argc != 6)
    {
        cerr << endl << "Usage: ./rgbd_inertial_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_imu" << endl;
        return 1;
    }

    cout << "[DEBUG] Vocabulary: " << argv[1] << endl;
    cout << "[DEBUG] Settings: " << argv[2] << endl;
    cout << "[DEBUG] Sequence path: " << argv[3] << endl;
    cout << "[DEBUG] Association file: " << argv[4] << endl;
    cout << "[DEBUG] IMU file: " << argv[5] << endl;

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestampsRGB;
    
    cout << "[DEBUG] Loading association file..." << endl;
    LoadImages(string(argv[4]), vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestampsRGB);

    // Check consistency in the number of images
    int nImages = vstrImageFilenamesRGB.size();
    cout << "[DEBUG] Loaded " << nImages << " image pairs" << endl;
    
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Load IMU data
    vector<double> vTimestampsImu;
    vector<cv::Point3f> vAcc, vGyro;
    
    cout << "[DEBUG] Loading IMU file..." << endl;
    LoadIMU(string(argv[5]), vTimestampsImu, vAcc, vGyro);

    cout << "[DEBUG] Total RGB-D images: " << nImages << endl;
    cout << "[DEBUG] Total IMU measurements: " << vTimestampsImu.size() << endl;

    // Create SLAM system in IMU-RGBD mode
    cout << "[DEBUG] Initializing ORB-SLAM3 system..." << endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true);
    float imageScale = SLAM.GetImageScale();
    
    cout << "[DEBUG] Image scale: " << imageScale << endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    
    int imuIndex = 0;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for(int ni=0; ni<nImages; ni++)
    {
        // Read image
        string rgbPath = string(argv[3])+"/"+vstrImageFilenamesRGB[ni];
        string depthPath = string(argv[3])+"/"+vstrImageFilenamesD[ni];
        
        if(ni == 0) {
            cout << "[DEBUG] Loading first RGB: " << rgbPath << endl;
            cout << "[DEBUG] Loading first depth: " << depthPath << endl;
        }
        
        imRGB = cv::imread(rgbPath, cv::IMREAD_UNCHANGED);
        imD = cv::imread(depthPath, cv::IMREAD_UNCHANGED);
        double tframe = vTimestampsRGB[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load RGB image at: " << rgbPath << endl;
            return 1;
        }
        
        if(imD.empty())
        {
            cerr << endl << "Failed to load depth image at: " << depthPath << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

        // Get IMU measurements between previous and current frame
        vImuMeas.clear();

        if(ni > 0)
        {
            double tframe_prev = vTimestampsRGB[ni-1];
            
            // Collect IMU data between frames
            while(imuIndex < vTimestampsImu.size() && vTimestampsImu[imuIndex] <= tframe)
            {
                if(vTimestampsImu[imuIndex] >= tframe_prev)
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                        vAcc[imuIndex].x, vAcc[imuIndex].y, vAcc[imuIndex].z,
                        vGyro[imuIndex].x, vGyro[imuIndex].y, vGyro[imuIndex].z,
                        vTimestampsImu[imuIndex]
                    ));
                }
                imuIndex++;
            }
        }

        if(ni % 50 == 0)
        {
            cout << "Frame " << ni << "/" << nImages << " - IMU measurements: " << vImuMeas.size() << endl;
        }

        // Pass the image to the SLAM system
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        
        if(vImuMeas.size() > 0)
            SLAM.TrackRGBD(imRGB, imD, tframe, vImuMeas);
        else
            SLAM.TrackRGBD(imRGB, imD, tframe);

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

        double ttrack= chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T = 0;
        if(ni < nImages-1)
            T = vTimestampsRGB[ni+1] - tframe;
        else if(ni > 0)
            T = tframe - vTimestampsRGB[ni-1];

        if(ttrack < T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    
    if(!fAssociation.is_open())
    {
        cerr << "Failed to open association file: " << strAssociationFilename << endl;
        return;
    }

    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation, s);
        
        if(!s.empty())
        {
            if(s[0] == '#')
                continue;
                
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, 
             vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    
    if(!fImu.is_open())
    {
        cerr << "Failed to open IMU file: " << strImuPath << endl;
        return;
    }

    vTimeStamps.reserve(50000);
    vAcc.reserve(50000);
    vGyro.reserve(50000);

    int line_count = 0;
    while(!fImu.eof())
    {
        string s;
        getline(fImu, s);
        
        if(s.empty() || s[0] == '#')
            continue;

        istringstream iss(s);
        double timestamp, gx, gy, gz, ax, ay, az;
        
        if(iss >> timestamp >> gx >> gy >> gz >> ax >> ay >> az)
        {
            vTimeStamps.push_back(timestamp);
            vGyro.push_back(cv::Point3f(gx, gy, gz));
            vAcc.push_back(cv::Point3f(ax, ay, az));
            line_count++;
        }
        else
        {
            cerr << "[WARNING] Failed to parse IMU line: " << s << endl;
        }
    }
    
    cout << "[DEBUG] Loaded " << line_count << " IMU measurements" << endl;
}