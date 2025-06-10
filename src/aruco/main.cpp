#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include "mqtt/async_client.h"

using namespace std;
using namespace cv;


const string server_adress{"mqtt://localhost:1883"};
const string client_id{"listener"};
const string topic{"ordre/commande"};


int main()
{
    //connection MQTT
    mqtt::async_client client(server_adress, client_id);
    mqtt::connect_options connOpts = mqtt::connect_options_builder()
                                     .mqtt_version(MQTTVERSION_DEFAULT)
                                     .automatic_reconnect()
                                     .clean_session()
                                     .finalize();



    try {
        // Connexion au broker
        cout << "Connexion au broker MQTT..." << endl;
        mqtt::token_ptr conntok = client.connect(connOpts);
        conntok->wait();
        cout << "Connecté !" << endl;

        // Créer un message à publier
        /*
        string payload = "programme Aruco"; 
        mqtt::message_ptr pubmsg = mqtt::make_message(topic, payload);
        pubmsg->set_qos(1);

        // Publier le message
        client.publish(pubmsg)->wait_for(std::chrono::seconds(10));
        cout << "Message publié sur " << topic << ": " << payload << endl;

        // Déconnexion propre
        //client.disconnect()->wait();
        //cout << "Déconnecté." << endl;


        */
    } catch (const mqtt::exception& exc) {
        cerr << "Erreur MQTT : " << exc.what() << endl;
        return 1;
    }


    // Charger les paramètres de calibration depuis un fichier YAML
    Mat cameraMatrix, distCoeffs;
    FileStorage fs("/home/pi/UnitreeRATP/src/aruco/calibration/camera_calibration.yml", FileStorage::READ);
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


    bool base = false;


    //position réelles des markers dans le monde en mètres

    map<int, Vec3d> markerWorldPositions = { // {x, y , z}
        {0, Vec3d(0.0, 0.0, 0.0)},
        {1, Vec3d(1.0, 0.0, 0.0)},
        {2, Vec3d(0.0,1.0,0.0)},
        // Ajoutez d'autres marqueurs selon leur placement
    };

    float markerLength = 0.1f;


        map<int, string> tag_ordre = {
        {10, "avancer $1000"},
        {11,"droite $90"},
        {12,"gauche $90"},
        {13,"reculer $1000"},
        };





    //*********************************************************************************

    // Initialisation de la carte
    const int mapWidth = 700;
    const int mapHeight = 700;
    Mat mapImage(mapHeight, mapWidth, CV_8UC3, Scalar(255, 255, 255));
    Mat mapImageCopy = mapImage.clone(); // Copie de l'image de fond


    // Paramètres de la carte
    const float scale = 100.0f; // 1 mètre = 100 pixels
    const Point2f mapOrigin(mapWidth / 2.0f, mapHeight / 2.0f); // Origine au centre



    // Dessiner les marqueurs sur la carte
    for (const auto& marker : markerWorldPositions) {
        Point2f pos = mapOrigin + Point2f(marker.second[0] * scale, -marker.second[1] * scale);
        circle(mapImage, pos, 5, Scalar(0, 0, 255), -1);
        putText(mapImage, "ID: " + to_string(marker.first), pos + Point2f(5, -5),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
    }


    //*********************************************************************************
    bool ok = false; 

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

            vector<Vec3d> cameraPositions;
            for (size_t i = 0; i < markerIds.size(); ++i) {
                int id = markerIds[i];

                //---------------détection tag ordre et envoie des commandes avec mqtt---------------

                auto ordre = tag_ordre.find(id);
                if(ordre != tag_ordre.end() && ok == false) {
                    cout << "tag ordre trouve: "<< id << endl;
                    cout << "commande envoyee: " << ordre->second << endl;
                    client.publish(topic, ordre->second);
                    ok = true; 
                    
                    
                    // Créer un message à publier
                    //string payload = ; 
                    //mqtt::message_ptr pubmsg = mqtt::make_message(topic, payload);
                    //pubmsg->set_qos(1);

                    // Publier le message
                    //client.publish(pubmsg)->wait_for(std::chrono::seconds(10));
                    //cout << "Message publié sur " << topic << ": " << payload << endl;

                }


                if (markerWorldPositions.find(id) == markerWorldPositions.end())
                    continue;

                // Définir les points 3D du marqueur dans son référentiel
                vector<Point3f> objectPoints = {
                    Point3f(-markerLength / 2.f,  markerLength / 2.f, 0),
                    Point3f( markerLength / 2.f,  markerLength / 2.f, 0),
                    Point3f( markerLength / 2.f, -markerLength / 2.f, 0),
                    Point3f(-markerLength / 2.f, -markerLength / 2.f, 0)
                };

                // Estimer la pose du marqueur
                Mat rvec, tvec;
                bool success = solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec);
                if (!success)
                    continue;

                // Dessiner les axes du marqueur
                drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, markerLength * 0.5f);

                // Calculer la distance entre le marqueur et la caméra
                double distance = norm(tvec) * 100.0; // en centimètres


                if(distance <= 30) {
                    client.publish(topic, "STOP");
                }

                // Calculer l'angle entre l'axe Z du marqueur et celui de la caméra
                Mat rotationMatrix;
                Rodrigues(rvec, rotationMatrix);
                Vec3d z_marker(rotationMatrix.at<double>(0,2), rotationMatrix.at<double>(1,2), rotationMatrix.at<double>(2,2));
                Vec3d z_camera(0, 0, 1);
                double angle_rad = acos(z_marker.dot(z_camera) / (norm(z_marker) * norm(z_camera)));
                double angle_deg = angle_rad * 180.0 / CV_PI;

                // Calculer la position de la caméra dans le référentiel du marqueur
                Mat cameraPosition = -rotationMatrix.t() * tvec;

                // Position du marqueur dans le monde
                Vec3d markerPosition = markerWorldPositions[id];

                // Position de la caméra dans le monde
                Vec3d cameraPositionWorld = markerPosition + Vec3d(cameraPosition);

                cameraPositions.push_back(cameraPositionWorld);

                // Afficher les informations sur l'image
                stringstream ss;
                ss << "ID: " << id
                   << " Dist: " << fixed << setprecision(2) << distance << "cm"
                   << " Angle Z: " << fixed << setprecision(2) << angle_deg << "deg"
                   << " Pos: (" << fixed << setprecision(2) << cameraPositionWorld[0]
                   << ", " << cameraPositionWorld[1] << ")";
                putText(frame, ss.str(), Point(10, 30 + 30 * i), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
            }

            // Calculer la position moyenne si plusieurs marqueurs sont détectés
            if (!cameraPositions.empty()) {
                Vec3d avgPosition(0, 0, 0);
                for (const auto& pos : cameraPositions)
                    avgPosition += pos;
                avgPosition /= static_cast<double>(cameraPositions.size());

                stringstream ss;
                ss << "Position moyenne: (" << fixed << setprecision(2) << avgPosition[0]
                   << ", " << avgPosition[1] << ")";
                putText(frame, ss.str(), Point(10, 30 + 30 * markerIds.size()), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);



                // Affichage de la position moyenne sur la carte
                mapImageCopy = mapImage.clone();
                Point2f avgCameraPosOnMap = mapOrigin + Point2f(avgPosition[0] * scale, -avgPosition[1] * scale);
                circle(mapImageCopy, avgCameraPosOnMap, 5, Scalar(0, 255, 0), -1);
                putText(mapImageCopy, "Camera (avg)", avgCameraPosOnMap + Point2f(5, -5),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);





                imshow("Carte en temps réel", mapImageCopy);


            }






        }

        imshow("Détection ArUco", frame);
        //namedWindow("camera", WINDOW_NORMAL);
        //resizeWindow("camera",200,200);
        //imshow("camera", frame);
        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();

    return 0;
}





























/*
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


    bool base = false;


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


        static int x, y = 0;

        if (!markerIds.empty()) {
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            cout << "IDs : ";
            for(int i=0; i< markerIds.size(); i++) {
                //cout << markerIds[i] << ", ";
                if(markerIds[i] == 0 && markerIds[i] == 1) {
                    cout << "robot actuellement à la position x: " << x << " y: " << y << endl;
                    base = true;

                    //affichage et vérification si on est bien à la base (x et y = 0)
                    if(markerIds[i] == 2) { //si on détecte le tag N°2 le robot commence sa ronde
                        cout << "depart du robot" << endl;
                    }
                }
            }
            //cout << endl; // remettre si on veut réafficher dans la console la liste des tags détectés







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
                    double rotation = norm(rvec); // angle en rad

                    double distanceCM = distance * 100; //distance en cm

                    constexpr int distLimite = 50;

                    //affichage d'un texte si un obstacle est détecté à 30cm ou moins sinon affichage RAS
                    if(distanceCM < distLimite) {
                        stringstream stop;
                        stop << "STOP obstacle detecte" << fixed << endl;
                        putText(frame, stop.str(),Point(10,90 + 30 * i),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,0,255),2);
                    }else {
                        stringstream go;
                        go << "RAS" << fixed << endl;
                        putText(frame, go.str(),Point(10,90 + 30 * i),FONT_HERSHEY_SIMPLEX,0.8,Scalar(0,255,0),2);

                    }



                    stringstream dist, rota;
                    dist << "distance: " << fixed << setprecision(2) << distanceCM << "cm" << endl; //fonctionnel
                    putText(frame, dist.str(),Point(10,30 + 30 * i),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,0,0),2);
                    rota << "rotation " << fixed << setprecision(2) << rotation * 180 /CV_PI << "deg" << endl; //fonctionnel
                    putText(frame, rota.str(),Point(10,60 + 30 * i),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,0,0),2);



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

                    /*
                    std::ostringstream oss;
                    oss << "Position: [" << cameraPosition.at<double>(0) << ", "
                        << cameraPosition.at<double>(1) << ", "
                        << cameraPosition.at<double>(2) << "] mm";
                    cv::putText(frame, oss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                                0.6, cv::Scalar(0, 255, 0), 2);

                    // remettre commentaire encadré





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
*/
