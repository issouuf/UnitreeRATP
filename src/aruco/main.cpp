#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include "mqtt/async_client.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace cv;

// --- Constantes MQTT ---
const string server_adress{"mqtt://localhost:1883"};
const string client_id{"follower_robot"};
const string topic{"ordre/commande"};


int main()
{
    // --- Connexion MQTT ---
    mqtt::async_client client(server_adress, client_id);
    mqtt::connect_options connOpts = mqtt::connect_options_builder()
                                     .mqtt_version(MQTTVERSION_DEFAULT)
                                     .automatic_reconnect()
                                     .clean_session()
                                     .finalize();
    try {
        cout << "Connexion au broker MQTT..." << endl;
        client.connect(connOpts)->wait();
        cout << "Connecté !" << endl;
    } catch (const mqtt::exception& exc) {
        cerr << "Erreur MQTT : " << exc.what() << endl;
        return 1;
    }

    // --- Initialisation de la caméra et de la calibration ---
    Mat cameraMatrix, distCoeffs;
    FileStorage fs("/home/pi/UnitreeRATP/src/aruco/calibration/camera_calibration.yml", FileStorage::READ);
    if (!fs.isOpened()) { cerr << "Erreur : impossible d'ouvrir le fichier de calibration." << endl; return -1; }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) { cerr << "Erreur : impossible d'ouvrir la caméra." << endl; return -1; }

    // --- Initialisation du détecteur ArUco ---
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_100);
    aruco::DetectorParameters detectorParams;
    aruco::ArucoDetector detector(dictionary, detectorParams);
    
    // La taille physique réelle de votre marqueur, en MÈTRES.
    float markerLength = 0.1f; // 10cm

    // --- NOUVEAU: Paramètres pour le mode "suivi" ---
    
    // L'ID du marqueur que le robot doit suivre. Changez cette valeur au besoin.
    const int FOLLOW_TAG_ID = 10; 
    
    // Distance (en mètres) à laquelle le robot s'arrêtera devant le tag.
    const float STOP_DISTANCE = 0.30f; // 30 cm
    
    // Seuil d'angle (en degrés). Si l'angle est supérieur, le robot tourne. Sinon, il avance.
    const float ANGLE_THRESHOLD_DEGREES = 3.0f;


    Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // --- Détection des marqueurs ---
        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        detector.detectMarkers(frame, markerCorners, markerIds);
        
        // On dessine tous les marqueurs détectés pour le débogage visuel
        if (!markerIds.empty()){
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        }

        // --- NOUVEAU: Logique de suivi de cible ---
        
        bool target_found = false;
        
        // On parcourt les IDs détectés pour trouver notre cible
        for (size_t i = 0; i < markerIds.size(); ++i) {
            if (markerIds[i] == FOLLOW_TAG_ID) {
                target_found = true;

                // --- On a trouvé notre cible, on estime sa pose ---
                vector<Point3f> objectPoints = {
                    Point3f(-markerLength / 2.f,  markerLength / 2.f, 0), Point3f( markerLength / 2.f,  markerLength / 2.f, 0),
                    Point3f( markerLength / 2.f, -markerLength / 2.f, 0), Point3f(-markerLength / 2.f, -markerLength / 2.f, 0)
                };

                Mat rvec, tvec;
                if (!solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec)) {
                    continue; // Si l'estimation échoue, on passe à l'image suivante
                }
                
                // On dessine les axes sur le tag cible pour mieux visualiser
                drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, markerLength);

                // --- On calcule la distance et l'angle par rapport au tag ---
                // tvec contient la position du tag dans le repère de la caméra.
                // tvec.at<double>(0) -> Axe X (gauche/droite)
                // tvec.at<double>(1) -> Axe Y (haut/bas)
                // tvec.at<double>(2) -> Axe Z (avant/arrière)

                double distance_m = tvec.at<double>(2); // La distance est la profondeur sur l'axe Z
                
                // L'angle est calculé à partir du déplacement sur l'axe X et de la distance Z
                double angle_rad = atan2(tvec.at<double>(0), tvec.at<double>(2));
                double angle_deg = angle_rad * 180.0 / M_PI;

                cout << "Cible ID " << FOLLOW_TAG_ID << " trouvée | "
                     << "Distance: " << fixed << setprecision(2) << distance_m << "m | "
                     << "Angle: " << fixed << setprecision(2) << angle_deg << " deg" << endl;

                // --- On génère la commande MQTT ---
                string command = "";

                // 1. D'abord, on corrige l'angle
                if (abs(angle_deg) > ANGLE_THRESHOLD_DEGREES) {
                    if (angle_deg < 0) { // Si l'angle est négatif, la cible est à droite
                        command = "droite $" + to_string(static_cast<int>(abs(angle_deg)));
                    } else { // Sinon, elle est à gauche
                        command = "gauche $" + to_string(static_cast<int>(-angle_deg));
                    }
                } 
                // 2. Si l'angle est bon, on ajuste la distance
                else if (distance_m > STOP_DISTANCE) {
                    // La commande avance d'une valeur proportionnelle à la distance restante
                    command = "avancer $" + to_string(static_cast<int>((distance_m - STOP_DISTANCE) * 100)); // en cm
                }
                // 3. Si on est bien aligné ET à la bonne distance, on s'arrête
                else {
                    command = "STOP";
                }
                
                cout << "Commande envoyée: " << command << endl;
                client.publish(topic, command);
                
                // On a trouvé et traité notre cible, on peut sortir de la boucle de recherche
                break; 
            }
        }

        // Si, après avoir parcouru tous les marqueurs, on n'a pas trouvé notre cible
        if (!target_found) {
            cout << "Cible ID " << FOLLOW_TAG_ID << " non visible. Arrêt du robot." << endl;
            client.publish(topic, "STOP");
        }


        // --- Affichage de la vidéo ---
        imshow("Robot Follower - Vue Caméra", frame);
        if (waitKey(1) == 'q') {
            break;
        }
    }

    // --- Nettoyage ---
    cout << "Arrêt du programme. Commande STOP finale." << endl;
    client.publish(topic, "STOP");
    cap.release();
    destroyAllWindows();
    client.disconnect()->wait();
    cout << "Déconnecté." << endl;

    return 0;
}


/*
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include "mqtt/async_client.h"

// NOUVEAU: Inclure pour M_PI si ce n'est pas déjà défini
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace cv;

// ... (vos constantes MQTT restent les mêmes)
const string server_adress{"mqtt://localhost:1883"};
const string client_id{"listener"};
const string topic{"ordre/commande"};


// NOUVEAU: Définir l'état du robot
enum RobotState {
    IDLE,              // En attente
    NAVIGATING,        // En train de se diriger vers une cible
    ARRIVED_AT_TARGET, // Vient d'arriver à la cible
    MISSION_COMPLETE   // A terminé tous les points du chemin
};

// NOUVEAU: Définir les types de marqueurs
enum MarkerType {
    LOCALIZATION_ONLY, // Marqueur utilisé uniquement pour se localiser
    WAYPOINT,          // Un point de passage dans une mission
    TASK_STATION       // Un point où une action spécifique doit être effectuée
};

// MODIFIÉ: Structure pour les informations des marqueurs
struct MarkerInfo {
    Vec3d position;        // Position 3D dans le monde (x, y, z)
    MarkerType type;       // Le type de marqueur
    std::string task_command; // Commande MQTT à exécuter si c'est une station de tâche
};

int main()
{
    // ... (la connexion MQTT reste la même)
    mqtt::async_client client(server_adress, client_id);
    mqtt::connect_options connOpts = mqtt::connect_options_builder()
                                     .mqtt_version(MQTTVERSION_DEFAULT)
                                     .automatic_reconnect()
                                     .clean_session()
                                     .finalize();
    try {
        cout << "Connexion au broker MQTT..." << endl;
        client.connect(connOpts)->wait();
        cout << "Connecté !" << endl;
    } catch (const mqtt::exception& exc) {
        cerr << "Erreur MQTT : " << exc.what() << endl;
        return 1;
    }

    // ... (le chargement de la calibration et l'ouverture de la caméra restent les mêmes)
    Mat cameraMatrix, distCoeffs;
    FileStorage fs("/home/pi/UnitreeRATP/src/aruco/calibration/camera_calibration.yml", FileStorage::READ);
    if (!fs.isOpened()) { cerr << "Erreur : impossible d'ouvrir le fichier de calibration." << endl; return -1; }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) { cerr << "Erreur : impossible d'ouvrir la caméra." << endl; return -1; }

    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_100);
    aruco::DetectorParameters detectorParams;
    aruco::ArucoDetector detector(dictionary, detectorParams);
    
    float markerLength = 0.1f; // 10cm

    // MODIFIÉ: Base de données des marqueurs connus dans le monde
    // Nous définissons ici la carte de l'environnement.
    // Les positions sont en mètres.
    map<int, MarkerInfo> markerDatabase = {
        // ID, {Position(x,y,z)}, Type, Commande de tâche
        {0, {Vec3d(0.0, 0.0, 0.0), TASK_STATION, "STOP"}}, // Base de départ / Arrêt d'urgence
        {10, {Vec3d(0.50, 4.0, 0.0), WAYPOINT, ""}},          // Waypoint 1
        //{11, {Vec3d(2.0, 1.5, 0.0), WAYPOINT, ""}},          // Waypoint 2
        //{12, {Vec3d(0.0, 1.5, 0.0), TASK_STATION, "action_speciale_1"}}, // Station de tâche
        {20, {Vec3d(-2.0, 1.0, 0.0), LOCALIZATION_ONLY, ""}},
        {21, {Vec3d(2.0, 1.0, 0.0), LOCALIZATION_ONLY, ""}},
        {22, {Vec3d(-2.0, 3.0, 0.0), LOCALIZATION_ONLY, ""}},
        {23, {Vec3d(2.0, 3.0, 0.0), LOCALIZATION_ONLY, ""}} // Marqueur juste pour se repérer
    };

    // NOUVEAU: Définir le plan de mission du robot
    // C'est la séquence d'IDs de WAYPOINT ou TASK_STATION que le robot doit suivre.
    vector<int> missionPath = {10,0}; // Va au wp 10, puis 11, puis 12, puis retourne à la base 0.
    
    // NOUVEAU: Variables d'état pour la navigation
    RobotState currentState = IDLE;
    int missionIndex = -1; // Index actuel dans missionPath
    Vec3d robotPose(0, 0, 0); // Position estimée du robot (x, y, heading_degrees)
    bool hasPosition = false; // Devient vrai si on a détecté au moins un marqueur

    // NOUVEAU: Paramètres de navigation
    const float arrival_threshold = 0.10f; // Seuil de distance pour considérer qu'on est arrivé (20cm)
    const float angle_threshold = 10.0f;   // Seuil d'angle pour considérer qu'on fait face à la cible (10 degrés)

    // ... (La partie initialisation de la carte pour l'affichage reste la même)
    const int mapWidth = 800;
    const int mapHeight = 800;
    Mat mapImage(mapHeight, mapWidth, CV_8UC3, Scalar(255, 255, 255));
    const float scale = 100.0f; // 1 mètre = 100 pixels
    const Point2f mapOrigin(mapWidth / 2.0f, mapHeight / 2.0f);

    for (const auto& marker : markerDatabase) {
        Point2f pos = mapOrigin + Point2f(marker.second.position[0] * scale, -marker.second.position[1] * scale);
        Scalar color = (marker.second.type == LOCALIZATION_ONLY) ? Scalar(128, 128, 128) : Scalar(0, 0, 255);
        circle(mapImage, pos, 5, color, -1);
        putText(mapImage, "ID: " + to_string(marker.first), pos + Point2f(5, -5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
    }
    Mat mapImageCopy;

    Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // --- DÉTECTION ET LOCALISATION (partie quasi inchangée) ---
        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        detector.detectMarkers(frame, markerCorners, markerIds);
        
        vector<Vec3d> cameraPositions;
        Vec3d robotOrientation(0,0,0); // On va aussi moyenner l'orientation
        int markers_used_for_pose = 0;

        if (!markerIds.empty()) {
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            for (size_t i = 0; i < markerIds.size(); ++i) {
                int id = markerIds[i];
                if (markerDatabase.find(id) == markerDatabase.end()) continue;

                vector<Point3f> objectPoints = {
                    Point3f(-markerLength / 2.f,  markerLength / 2.f, 0), Point3f( markerLength / 2.f,  markerLength / 2.f, 0),
                    Point3f( markerLength / 2.f, -markerLength / 2.f, 0), Point3f(-markerLength / 2.f, -markerLength / 2.f, 0)
                };

                Mat rvec, tvec;
                if (!solvePnP(objectPoints, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec)) continue;
                
                drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, markerLength * 0.5f);

                Mat rotationMatrix;
                Rodrigues(rvec, rotationMatrix);
                
                // Position et orientation de la caméra par rapport au marqueur
                Mat cameraPositionInMarkerFrame = -rotationMatrix.t() * tvec;
                Mat cameraRotationInMarkerFrame = rotationMatrix.t();

                // NOUVEAU: Calcul de l'orientation (cap) du robot
                // On suppose que le robot regarde dans la direction Z de la caméra
                // On calcule l'angle de cette direction dans le plan XY du monde
                Vec3d z_axis_cam_in_marker_frame = {cameraRotationInMarkerFrame.at<double>(0,2), cameraRotationInMarkerFrame.at<double>(1,2), cameraRotationInMarkerFrame.at<double>(2,2)};
                // Pour simplifier, on ignore l'orientation du marqueur lui-même (on suppose qu'ils sont tous alignés avec les axes du monde)
                // Une version avancée prendrait en compte la rotation de chaque marqueur dans le monde.
                
                Vec3d markerPositionInWorld = markerDatabase[id].position;
                Vec3d cameraPositionInWorld = markerPositionInWorld + Vec3d(cameraPositionInMarkerFrame);
                
                cameraPositions.push_back(cameraPositionInWorld);

                // Calcul du cap (heading) en degrés. atan2(y, x)
                double heading_rad = atan2(z_axis_cam_in_marker_frame[1], z_axis_cam_in_marker_frame[0]);
                robotOrientation += Vec3d(0, 0, heading_rad * 180.0 / M_PI); // On ne s'intéresse qu'au cap (autour de Z)
                markers_used_for_pose++;
            }
        }

        hasPosition = false;
        if (markers_used_for_pose > 0) {
            Vec3d avgPosition(0, 0, 0);
            for (const auto& pos : cameraPositions) avgPosition += pos;
            avgPosition /= (double)markers_used_for_pose;
            
            Vec3d avgOrientation = robotOrientation / (double)markers_used_for_pose;
            
            robotPose = Vec3d(avgPosition[0], avgPosition[1], avgOrientation[2]);
            hasPosition = true;
        }

        // --- NOUVEAU: LOGIQUE DE NAVIGATION AUTONOME ---
        if (hasPosition) {
            // Si on est en attente (IDLE), on commence la mission
            if (currentState == IDLE) {
                cout << "Lancement de la mission..." << endl;
                missionIndex = 0;
                currentState = NAVIGATING;
            }

            if (currentState == NAVIGATING) {
                if (missionIndex >= missionPath.size()) {
                    cout << "Mission terminée !" << endl;
                    client.publish(topic, "STOP");
                    currentState = MISSION_COMPLETE;
                } else {
                    int targetId = missionPath[missionIndex];
                    Vec3d targetPose = markerDatabase[targetId].position;

                    // Calcul du vecteur vers la cible
                    Vec3d vectorToTarget = targetPose - Vec3d(robotPose[0], robotPose[1], 0);
                    double distanceToTarget = norm(vectorToTarget);

                    cout << "Cible: ID " << targetId << " | Distance: " << fixed << setprecision(2) << distanceToTarget << "m" << endl;

                    // 1. Vérifier si on est arrivé
                    if (distanceToTarget < arrival_threshold) {
                        cout << "Arrivé à la cible ID " << targetId << endl;
                        currentState = ARRIVED_AT_TARGET;
                    } else {
                        // 2. Si non, on navigue: s'orienter puis avancer
                        double angleToTarget_rad = atan2(vectorToTarget[1], vectorToTarget[0]);
                        double angleToTarget_deg = angleToTarget_rad * 180.0 / M_PI;

                        double headingError_deg = angleToTarget_deg - robotPose[2];
                        
                        // Normaliser l'erreur d'angle entre -180 et 180
                        while (headingError_deg > 180) headingError_deg -= 360;
                        while (headingError_deg < -180) headingError_deg += 360;

                        cout << "Cap robot: " << robotPose[2] << " | Cap cible: " << angleToTarget_deg << " | Erreur: " << headingError_deg << endl;
                        
                        string command = "";
                        // 2a. D'abord, corriger l'orientation
                        if (abs(headingError_deg) > angle_threshold) {
                            if (headingError_deg > 0) {
                                command = "gauche $" + to_string(static_cast<int>(headingError_deg));
                            } else {
                                command = "droite $" + to_string(static_cast<int>(abs(headingError_deg)));
                            }
                        } else {
                        // 2b. Une fois bien orienté, avancer
                            command = "avancer $" + to_string(static_cast<int>(distanceToTarget * 100)); // ex: avancer $150 (cm)
                        }

                        cout << "Commande envoyee: " << command << endl;
                        client.publish(topic, command);
                    }
                }
            }
            
            if (currentState == ARRIVED_AT_TARGET) {
                 client.publish(topic, "STOP"); // S'arrêter en arrivant
                
                int currentTargetId = missionPath[missionIndex];
                MarkerInfo targetInfo = markerDatabase[currentTargetId];

                // Si c'est une station de tâche, exécuter la commande
                if(targetInfo.type == TASK_STATION && !targetInfo.task_command.empty()){
                    cout << "Execution de la tache: " << targetInfo.task_command << endl;
                    client.publish(topic, targetInfo.task_command);
                    // Mettre une pause pour laisser le temps à la tâche de s'exécuter
                    this_thread::sleep_for(chrono::seconds(3));
                }

                // Passer à la cible suivante
                missionIndex++;
                currentState = NAVIGATING;
            }
        } else {
             cout << "En attente de localisation..." << endl;
        }

        // --- AFFICHAGE (partie quasi inchangée) ---
        if(hasPosition){
             mapImageCopy = mapImage.clone();
             Point2f robotPosOnMap = mapOrigin + Point2f(robotPose[0] * scale, -robotPose[1] * scale);
             circle(mapImageCopy, robotPosOnMap, 7, Scalar(0, 255, 0), -1); // Robot en vert
             
             // Dessiner le cap du robot
             Point2f headingEndPoint;
             headingEndPoint.x = robotPosOnMap.x + 20 * cos(robotPose[2] * M_PI / 180.0);
             headingEndPoint.y = robotPosOnMap.y - 20 * sin(robotPose[2] * M_PI / 180.0); // Y inversé sur la carte
             line(mapImageCopy, robotPosOnMap, headingEndPoint, Scalar(255, 0, 0), 2); // Cap en bleu

             imshow("Carte en temps réel", mapImageCopy);
        }

        imshow("Détection ArUco", frame);
        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    client.disconnect()->wait();
    cout << "Déconnecté." << endl;

    return 0;
}



*/
























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
