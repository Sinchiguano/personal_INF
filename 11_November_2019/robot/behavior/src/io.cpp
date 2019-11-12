#include <iostream>
#include <tuple>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "io.hpp"

void create_dir (std::string const &dirname) {
    boost::filesystem::create_directories(dirname);
}

std::string get_writable_path(std::string const &path) {
    boost::filesystem::path boost_path(path);
    auto parent = boost_path.parent_path();
    boost::filesystem::create_directories(parent);
    return path;
}

std::string basename(std::string const &pathname) {
    boost::filesystem::path p(pathname);
    return p.filename().string();
}
std::string stem_path(std::string const &pathname) {
    boost::filesystem::path p(pathname);
    return p.stem().string();
}

std::string get_counted_fname (
        int cnt, std::string const &prefix,
        std::string const &suffix) {
    std::stringstream stream;
    stream << prefix << std::setw(4) << std::setfill('0') << cnt << suffix;
    return stream.str();
}
