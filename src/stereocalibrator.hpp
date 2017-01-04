#ifndef STEREOCALIBRATOR_H
#define STEREOCALIBRATOR_H

#include "global_includes.hpp"

class StereoCalibrator
{

public:

    /**
     * @brief StereoCalibrator class constructor
     * @param chessboardSize size of the calibration chessboard pattern
     */
    StereoCalibrator(cv::Size &chessboardSize);

    /**
     * @brief setChessboardSize modifyies chessboard pattern size
     * @param chessboardSize new chessboard pattern size
     */
    void setChessboardSize(cv::Size &chessboardSize);

    /**
     * @brief compute given two sequences of images containing a calibration chessboard
     *        pattern computes intrinsics matrix, distortion coefficients, rectification
     *        transform matrix, projection matrix in the rectified coordinate system and
     *        disparity-to-depth mapping matrix for both left and right camera.
     * @param left calibration images sequence from left camera.
     * @param right calibration images sequence from right camera.
     * @param skipCheck if true checkVectors is called before calibration.
     * @return true if calibration succeeded without errors.
     */
    double compute(std::vector<cv::Mat> &left, std::vector<cv::Mat> &right, bool skipCheck = true);

    /**
     * @brief saveCalibration saves calibration paramteres in an xml file.
     * @param fileName path to file.
     * @return true if no errors occurred.
     */
    bool saveCalibration(const std::string &fileName);

    /**
     * @brief loadCalibration loads calibration parameteres from an xml file
     *        created by saveCalibration.
     * @param fileName path to file.
     * @return true if no errors occurred.
     */
    bool loadCalibration (const std::string &fileName);

    /**
     * @brief checkVectors checks if two calibration images sequences are not empty,
     *        are of the same length and images have the same size.
     * @param left calibration images sequence from left camera.
     * @param right calibration images sequence from right camera.
     * @return false if one of the sequences is empty or if images
     *         are of different sizes. True otherwise.
     */
    static bool checkVectors(std::vector<cv::Mat> &left, std::vector<cv::Mat> &right);

    /**
     * @brief getChessboardPattern builds a chessboard inner corners pattern for a
     *        chessboard of the given size
     * @param chessboardSize given chessboard size
     * @param pattern output pattern. The points are placed in row-major left-to-right order
     */
    static void getChessboardPattern(const cv::Size &chessboardSize, std::vector<cv::Point3f> &pattern);

private:

    cv::Size chessboardSize;
    cv::Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
    cv::Mat R1, R2, P1, P2, Q;
};

#endif // STEREOCALIBRATOR_H
