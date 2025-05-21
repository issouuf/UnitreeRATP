#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

using namespace std;
using namespace cv;

int main()
{
    // Charger les paramètres de calibration depuis un fichier YAML
    Mat cameraMatrix, distCoeffs;
    FileStorage fs("/Users/ulysse/Documents/unitree/camera_calibration.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Erreur : impossible d'ouvrir le fichier de calibration." << endl;
        return -1;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    // Ouvrir la caméra
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Erreur : impossible d'ouvrir la caméra." << endl;
        return -1;
    }

    // Initialiser le détecteur ArUco
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_100);
    aruco::DetectorParameters detectorParams;
    aruco::ArucoDetector detector(dictionary, detectorParams);

    Mat frame, gray;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cerr << "Erreur : image vide capturée." << endl;
            break;
        }

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners, rejectedCandidates;

        detector.detectMarkers(gray, markerCorners, markerIds);

        if (!markerIds.empty()) {
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            cout << "IDs : ";
            for(int i=0; i< markerIds.size(); i++) {
                cout << markerIds[i] << ", ";
            }
            cout << endl;


            for (size_t i = 0; i < markerIds.size(); ++i) {
                // Définir les points 3D du marqueur dans son référentiel
                float markerLength = 0.1f; // en mètres
                vector<Point3f> objectPoints = {
                    Point3f(-markerLength / 2.f,  markerLength / 2.f, 0),
                    Point3f( markerLength / 2.f,  markerLength / 2.f, 0),
                    Point3f( markerLength / 2.f, -markerLength / 2.f, 0),
                    Point3f(-markerLength / 2.f, -markerLength / 2.f, 0)
                };

                // Estimer la pose du marqueur
                Mat rvec, tvec;
                bool success = solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec);
                if (success) {
                    // Dessiner les axes du marqueur
                    drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, markerLength * 0.5f);

                    //putText ordre des couleurs: (B,G,R)


                    //calcul de la distance entre un tag et la caméra en m

                    double distance = norm(tvec); // distance en m
                    double rotation = norm(rvec); //

                    double distanceCM = distance * 100;

                    cout << distanceCM << endl;

                    //affichage d'un texte si un obstacle est détecté à 30cm ou moins sinon affichage RAS
                    if(distanceCM < 30) {
                        stringstream stop;
                        stop << "STOP obstacle detecte" << fixed << endl;
                        putText(frame, stop.str(),Point(10,90 + 30 * i),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,0,255),2);
                    }else {
                        stringstream go;
                        go << "RAS" << fixed << endl;
                        putText(frame, go.str(),Point(10,90 + 30 * i),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0),2);

                    }



                    stringstream dist;
                    dist << "distance: " << fixed << setprecision(2) << distanceCM << "cm" << endl; //fonctionnel
                    //dist << "rotation " << fixed << setprecision(2) << rotation * 180 /CV_PI << "deg" << endl; //fonctionnel
                    putText(frame, dist.str(),Point(10,60 + 30 * i),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,0,0),2);



                    Mat rotationMatrix;

                    Rodrigues(rvec,rotationMatrix);



                    Mat cameraPosition;

                    cameraPosition = -rotationMatrix.t() * tvec;



                    Mat R;
                    Rodrigues(rvec,R);

                    Vec3d z_marker(R.at<double>(0,2),R.at<double>(1,2),R.at<double>(2,2));

                    Vec3d z_camera(0,0,1);

                    //calcul de l'angle entre les deux vecteurs Z

                    double dot_product = z_marker.dot(z_camera);
                    double angle_rad = acos(dot_product);
                    double angle_deg = angle_rad * 180 / CV_PI;

                    //affichage de l'angle sur l'image
                    stringstream ss;
                    ss << "angle Z: " << fixed << setprecision(2) << angle_deg << "deg" << endl;
                    //putText(frame, ss.str(),Point(10,30 + 30 * i),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,0,0),2);


                    std::ostringstream oss;
                    oss << "Position: [" << cameraPosition.at<double>(0) << ", "
                        << cameraPosition.at<double>(1) << ", "
                        << cameraPosition.at<double>(2) << "] mm";
                    cv::putText(frame, oss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                                0.6, cv::Scalar(0, 255, 0), 2);







                }
            }
        }

        imshow("Détection ArUco", frame);
        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();

    return 0;
}
