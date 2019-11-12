#ifndef IO_HPP
#define IO_HPP

#include <tuple>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

// create directory if it does not exist
void create_dir (std::string const &dirname);

// create directory for file path if it does not exist
std::string get_writable_path(std::string const &path);

std::string get_counted_fname (
        int cnt, std::string const &prefix,
        std::string const &suffix);

std::string basename(std::string const &pathname);
// /foo/bar.txt -> bar
std::string stem_path(std::string const &pathname);

#endif /* ifndef IO_HPP */

