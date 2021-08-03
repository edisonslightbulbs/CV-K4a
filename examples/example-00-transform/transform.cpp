#include <opencv2/opencv.hpp>

// OpenCV size formats
//     f(x)  *   f(y)
//    width  *   height
//     cols  *   rows

// for scaling we use:  cv.resize()
//  -  size of image can be specified manually
//  -  size of image can be specified using a scaling factor
//  -  interpolation methods used
//     1.  cv.INTER_AREA
//     2.  cv.INTER_CUBIC
//     3.  cv.INTER_LINEAR
//
void scale()
{
    // initialize resources
    const std::string INPUT = "scale-input";
    const std::string OUTPUT = "scale-output";
    cv::Mat src, dst;

    src = cv::imread("./resources/sample.png");

    // scale
    cv::Size dSize = cv::Size(300, 300);
    cv::resize(src, dst, dSize, 0, 0, cv::INTER_AREA);

    // show window
    cv::imshow(INPUT, src);
    cv::imshow(OUTPUT, dst);
    cv::waitKey();
}

void transform()
{
    // initialize resources
    const std::string INPUT = "transform-input";
    const std::string OUTPUT = "transform-output";

    cv::Mat src, dst, M;
    src = cv::imread("./resources/sample.png");
    cv::Size dSize = cv::Size(src.rows, src.cols);

    // transform
    double arr[6] = { 1, 0, 50, 0, 1, 100 };
    M = cv::Mat(2, 3, CV_64FC1, arr);
    cv::warpAffine(src, dst, M, dSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
        cv::Scalar());

    // show window
    cv::imshow(INPUT, src);
    cv::imshow(OUTPUT, dst);
    cv::waitKey();
}

void rotate()
{
    // initialize resources
    const std::string INPUT = "rotate-input";
    const std::string OUTPUT = "rotate-output";

    cv::Mat src, dst, M;
    src = cv::imread("./resources/sample.png");
    cv::Size dSize = cv::Size(src.rows, src.cols);

    // transform
    cv::Point center = cv::Point(src.cols / 2, src.rows / 2);
    M = cv::getRotationMatrix2D(center, 45, 1);
    cv::warpAffine(src, dst, M, dSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
        cv::Scalar());

    // show window
    cv::imshow(INPUT, src);
    cv::imshow(OUTPUT, dst);
    cv::waitKey();
}

void affine()
{
    // initialize resources
    const std::string INPUT = "affine-transformation-input";
    const std::string OUTPUT = "affine-transformation-output";

    cv::Mat src, dst, M;
    src = cv::imread("./resources/sample.png");
    cv::Size dSize = cv::Size(src.rows, src.cols);

    float srcArr[6] = { 0, 0, 0, 1, 1, 0 };
    float destArr[6] = { 0.6, 0.2, 0.1, 1.3, 1.5, 0.3 };

    // transform
    cv::Mat srcTri = cv::Mat(3, 1, CV_32FC2, srcArr);
    cv::Mat dstTri = cv::Mat(3, 1, CV_32FC2, destArr);
    M = cv::getAffineTransform(srcTri, dstTri);
    cv::warpAffine(src, dst, M, dSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
        cv::Scalar());

    // show window
    cv::imshow(INPUT, src);
    cv::imshow(OUTPUT, dst);
    cv::waitKey();
}

void perspective()
{
    // initialize resources
    const std::string INPUT = "perspective-transformation-input";
    const std::string OUTPUT = "perspective-transformation-output";

    cv::Mat src, dst, M;
    src = cv::imread("./resources/sample.png");
    cv::Size dSize = cv::Size(src.rows, src.cols);

    float srcArr[8] = { 56, 65, 368, 52, 28, 387, 389, 390 };
    float destArr[8] = { 0, 0, 300, 0, 0, 300, 300, 300 };

    // transform
    cv::Mat srcTri = cv::Mat(3, 1, CV_32FC2, srcArr);
    cv::Mat dstTri = cv::Mat(3, 1, CV_32FC2, destArr);
    M = cv::getAffineTransform(srcTri, dstTri);
    cv::warpAffine(src, dst, M, dSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
        cv::Scalar());

    // show window
    cv::imshow(INPUT, src);
    cv::imshow(OUTPUT, dst);
    cv::waitKey();
}

int main()
{
    scale();
    transform();
    rotate();
    affine();
    perspective();
    return 0;
}
