#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/objdetect/aruco_board.hpp>

#include <vector>
#include <iostream>
#include <fstream>

// --- Paramètres de VOTRE Charuco Board (À AJUSTER ABSOLUMENT) ---
const int CHARUCO_SQUARES_X = 5;
const int CHARUCO_SQUARES_Y = 7;
const float CHARUCO_SQUARE_LENGTH = 0.040f;
const float CHARUCO_MARKER_LENGTH = 0.020f;
const int CHARUCO_DICTIONARY_ID_ENUM_VAL = cv::aruco::DICT_6X6_250;

// --- Fichier de sauvegarde des paramètres de calibration ---
const std::string CALIBRATION_FILENAME = "camera_calibration.yml";

// --- Nombre minimum de vues pour la calibration ---
const int MIN_CALIBRATION_FRAMES = 15;

bool saveCameraCoefficients(const std::string& filename, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, double reprojectionError) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open file for writing: " << filename << std::endl;
        return false;
    }
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "avg_reprojection_error" << reprojectionError;
    fs.release();
    std::cout << "Camera coefficients saved to " << filename << std::endl;
    return true;
}

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // 1. Dictionnaire ArUco
    // getPredefinedDictionary retourne un Ptr<Dictionary>, c'est la méthode standard.
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(CHARUCO_DICTIONARY_ID_ENUM_VAL);


    // 2. CharucoBoard
    // CharucoBoard::create retourne un Ptr<CharucoBoard>.
   cv::aruco::CharucoBoard charucoBoard =
        cv::aruco::CharucoBoard(
            cv::Size(CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
            CHARUCO_SQUARE_LENGTH,
            CHARUCO_MARKER_LENGTH,
            dictionary); // Attend un Ptr<Dictionary>

    // 3. Paramètres pour la détection des marqueurs ArUco (objet sur la pile)
    cv::aruco::DetectorParameters arucoParams; // Création directe, pas via Ptr
    // Vous pouvez ajuster les paramètres ici si nécessaire:
    // arucoParams.minMarkerPerimeterRate = 0.01;
    // arucoParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    // 4. Paramètres pour la détection Charuco (objet sur la pile)
    cv::aruco::CharucoParameters charucoParams; // Création directe
    // Vous pouvez ajuster les paramètres Charuco ici si nécessaire:
    // charucoParams.tryRefineMarkers = true;

    // 5. Création du CharucoDetector
    // Le constructeur de CharucoDetector prend la planche (par référence),
    // les paramètres Charuco (par référence), et les paramètres du détecteur ArUco (par référence).
    // Nous devons déréférencer charucoBoard (qui est un Ptr) pour obtenir la référence.
    cv::aruco::CharucoDetector charucoDetector(
        charucoBoard,    // Déréférencer le Ptr<CharucoBoard> pour passer CharucoBoard&
        charucoParams,    // Passer l'objet CharucoParameters directement (c'est déjà une référence implicite)
        arucoParams       // Passer l'objet DetectorParameters directement
        );

    std::vector<std::vector<cv::Point2f>> allCharucoCorners;
    std::vector<std::vector<int>> allCharucoIds;
    cv::Size imageSize;

    std::cout << "Starting calibration process (OpenCV 4.8.0 API, min Ptr usage)..." << std::endl;
    std::cout << "Show the Charuco board to the camera from different angles and distances." << std::endl;
    std::cout << "Press 'c' to capture a frame for calibration." << std::endl;
    std::cout << "Press 'q' or ESC to finish capturing and start calibration (needs at least " << MIN_CALIBRATION_FRAMES << " frames)." << std::endl;

    int capturedFramesCount = 0;

    while (true) {
        cv::Mat frame, frameCopy, gray;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed." << std::endl;
            break;
        }
        frame.copyTo(frameCopy);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        if (imageSize.empty()) {
            imageSize = gray.size();
        }

        std::vector<int> currentCharlucoIds;
        std::vector<cv::Point2f> currentCharlucoCorners;
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        charucoDetector.detectBoard(gray, currentCharlucoCorners, currentCharlucoIds, markerCorners, markerIds);

        if (!currentCharlucoIds.empty() && currentCharlucoCorners.size() > 3) {
            cv::aruco::drawDetectedCornersCharuco(frameCopy, currentCharlucoCorners, currentCharlucoIds, cv::Scalar(255, 0, 0));
            if(!markerIds.empty()) {
                cv::aruco::drawDetectedMarkers(frameCopy, markerCorners, markerIds);
            }

            char key = (char)cv::waitKey(30);
            if (key == 'c') {
                if (currentCharlucoIds.size() >= 4) {
                    allCharucoCorners.push_back(currentCharlucoCorners);
                    allCharucoIds.push_back(currentCharlucoIds);
                    capturedFramesCount++;
                    std::cout << "Frame captured. Total frames: " << capturedFramesCount << "/" << MIN_CALIBRATION_FRAMES << std::endl;
                    if (capturedFramesCount >= MIN_CALIBRATION_FRAMES) {
                        std::cout << "Sufficient frames captured. Press 'q' or ESC to calibrate, or 'c' for more." << std::endl;
                    }
                } else {
                    std::cout << "Not enough Charuco corners detected in this view (" << currentCharlucoIds.size() << "), try a different angle." << std::endl;
                }
            } else if (key == 'q' || key == 27) {
                if (capturedFramesCount >= MIN_CALIBRATION_FRAMES) {
                    break;
                } else {
                    std::cout << "Need at least " << MIN_CALIBRATION_FRAMES << " frames to calibrate. Currently have " << capturedFramesCount << "." << std::endl;
                }
            }
        }

        cv::imshow("Camera Calibration - Press 'c' to capture, 'q' or ESC to finish", frameCopy);
        char key_wait = (char)cv::waitKey(10);
        if ((key_wait == 'q' || key_wait == 27) && capturedFramesCount >= MIN_CALIBRATION_FRAMES) {
            break;
        }
        if (key_wait == 27 && capturedFramesCount < MIN_CALIBRATION_FRAMES) {
            std::cout << "Calibration aborted by user." << std::endl;
            cap.release();
            cv::destroyAllWindows();
            return 0;
        }
    }
    cv::destroyWindow("Camera Calibration - Press 'c' to capture, 'q' or ESC to finish");

    if (allCharucoCorners.size() < MIN_CALIBRATION_FRAMES) {
        std::cerr << "Not enough frames for calibration. Acquired " << allCharucoCorners.size()
        << ", but need at least " << MIN_CALIBRATION_FRAMES << "." << std::endl;
        cap.release();
        return -1;
    }

    std::cout << "\nStarting camera calibration with " << capturedFramesCount << " frames..." << std::endl;

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    int calibrationFlags = 0;

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    for (size_t i = 0; i < allCharucoCorners.size(); ++i) {
        cv::Mat objPts, imgPts;
        charucoBoard.matchImagePoints(allCharucoCorners[i], allCharucoIds[i], objPts, imgPts);
        if (objPts.total() > 0 && imgPts.total() > 0) {
            std::vector<cv::Point3f> objVec(objPts.begin<cv::Point3f>(), objPts.end<cv::Point3f>());
            std::vector<cv::Point2f> imgVec(imgPts.begin<cv::Point2f>(), imgPts.end<cv::Point2f>());
            objectPoints.push_back(objVec);
            imagePoints.push_back(imgVec);
        }
    }

    double reprojectionError = cv::calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs,
        calibrationFlags
        );

    std::cout << "Calibration finished." << std::endl;
    std::cout << "Average Reprojection Error: " << reprojectionError << std::endl;
    std::cout << "Camera Matrix (fx, fy, cx, cy): \n" << cameraMatrix << std::endl;
    std::cout << "Distortion Coefficients (k1, k2, p1, p2, k3, k4, k5, k6 ...): \n" << distCoeffs.t() << std::endl;

    if (saveCameraCoefficients(CALIBRATION_FILENAME, cameraMatrix, distCoeffs, reprojectionError)) {
        std::cout << "Successfully saved coefficients to " << CALIBRATION_FILENAME << std::endl;
    } else {
        std::cerr << "Error saving coefficients." << std::endl;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
