#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <set>
#include <string>
#include <unistd.h>

#include "io.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

std::string io::pwd()
{
    char buff[FILENAME_MAX];
    getcwd(buff, FILENAME_MAX);
    std::string workingDir(buff);
    return workingDir;
}
#pragma GCC diagnostic pop

std::vector<Point> io::read(std::vector<Point> points, const char* file)
{
    /** create input stream and string for parsing file data */
    std::ifstream data(file);
    std::string line;

    /** assign each parsed point and id starting from 1 */
    int id = 1;

    /** while un-parsed lines exist ... */
    while (std::getline(data, line)) {

        /** ... get next line in file */
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        /** ... parse each line based on delimiter */
        while (std::getline(ss, cell, ' ')) {
            row.push_back(cell);
        }

        /** create Point type definitions */
        Point point((float)std::stof(row[0]), (float)std::stof(row[1]),
            (float)std::stof(row[2]));
        points.push_back(point);
        id++;
    }
    return points;
}

void io::write(std::vector<float>& y, const std::string& file)
{
    std::ofstream filestream;
    filestream.open(file);
    filestream << "x,y" << std::endl;
    int x = 1;
    for (const auto& val : y) {
        filestream << x << ", " << val << std::endl;
        x++;
    }
    filestream.close();
}

void io::write(const k4a_image_t& t_rgb)
{
    const std::string IMAGE_FILE = "./output/scene.png";
    int rgbWidth = k4a_image_get_width_pixels(t_rgb);
    int rgbHeight = k4a_image_get_height_pixels(t_rgb);

    uint8_t* color_image_data = k4a_image_get_buffer(t_rgb);
    cv::Mat m_image = cv::Mat(rgbHeight, rgbWidth, CV_8UC4,
        (void*)color_image_data, cv::Mat::AUTO_STEP);

    cv::imwrite(IMAGE_FILE, m_image);
}

void io::performance(const float& rawData, const float& filteredData,
    const std::string& filterTime, const float& coarseSeg,
    const std::string& coarseSegTime, const float& finalSeg,
    const std::string& finalSegTime, const std::string& totalRuntime)
{
    std::string OUTPUT_PATH = io::pwd() + "/build/bin/runtime.csv";

    /** output pipeline operation runtimes */
    std::ofstream filestream;
    filestream.open(OUTPUT_PATH, std::ofstream::app);
    filestream << rawData << ", " << filteredData << ", " << filterTime << ", "
               << coarseSeg << ", " << coarseSegTime << ", " << finalSeg << ", "
               << finalSegTime << ", " << totalRuntime << std::endl;

    /** [%] data reduction at each filter */
    const float TO_PERCENT = 100;
    float duringOutlierRemoval
        = ((rawData - filteredData) / rawData) * TO_PERCENT;
    float duringCoarseSeg = ((filteredData - coarseSeg) / rawData) * TO_PERCENT;
    float duringFinalSeg = ((coarseSeg - finalSeg) / rawData) * TO_PERCENT;
    float totalReduction
        = duringOutlierRemoval + duringCoarseSeg + duringFinalSeg;

    /** output data reduction */
    OUTPUT_PATH = io::pwd() + "/output/reduction.csv";
    filestream.open(OUTPUT_PATH, std::ofstream::app);
    filestream << duringOutlierRemoval << ", " << duringCoarseSeg << ", "
               << duringFinalSeg << ", " << totalReduction << std::endl;
    io::sortLog();
}

void io::sortLog()
{
    const std::string INPUT_PATH = io::pwd() + "/build/bin/runtime.csv";
    const std::string OUTPUT_PATH = io::pwd() + "/output/runtime.csv";

    std::ifstream data(INPUT_PATH);
    std::string line;

    std::vector<std::vector<float>> results;
    std::vector<float> temp;

    /** while un-parsed lines exist ... */
    while (std::getline(data, line)) {

        /** ... get next line in file */
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        /** ... parse each line based on delimiter */
        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        temp.push_back((float)std::stof(row[0]));
        temp.push_back((float)std::stof(row[1]));
        temp.push_back((float)std::stof(row[2]));
        temp.push_back((float)std::stof(row[3]));
        temp.push_back((float)std::stof(row[4]));
        temp.push_back((float)std::stof(row[5]));
        temp.push_back((float)std::stof(row[6]));
        temp.push_back((float)std::stof(row[7]));
        results.push_back(temp);
        temp.clear();
    }

    /** sort results using total runtime (index 0) as a basis */
    std::sort(results.begin(), results.end(),
        [](const std::vector<float>& a, const std::vector<float>& b) {
            return a[0] < b[0];
        });

    /** write sorted logs onto output file */
    std::ofstream filestream;
    filestream.open(OUTPUT_PATH);
    for (auto list : results) {
        for (int i = 0; i < list.size(); i++) {
            if (i == list.size() - 1) {
                filestream << list[i] << std::endl;
            } else {
                filestream << list[i] << ", ";
            }
        }
    }
    filestream.close();
}

template <typename T>
std::pair<bool, int> getIndex(const std::vector<T>& vec, const T& element)
{
    std::pair<bool, int> result;

    auto it = std::find(vec.begin(), vec.end(), element);
    if (it != vec.end()) {
        result.second = distance(vec.begin(), it);
        result.first = true;
    }

    else {
        result.first = false;
        result.second = -1;
    }
    return result;
}
