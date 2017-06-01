#include "wave/utils/file.hpp"

namespace wave {

bool file_exists(const std::string &fp) {
    FILE *file;

    if ((file = fopen(fp.c_str(), "r"))) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

std::vector<std::string> path_split(const std::string path) {
    std::string s;
    std::vector<std::string> splits;

    s = "";
    for (size_t i = 0; i < path.length(); i++) {
        if (s != "" && path[i] == '/') {
            splits.push_back(s);
            s = "";
        } else if (path[i] != '/') {
            s += path[i];
        }
    }
    splits.push_back(s);

    return splits;
}

void paths_combine(const std::string path1,
                   const std::string path2,
                   std::string &out) {
    int dirs_up;
    std::vector<std::string> splits1;
    std::vector<std::string> splits2;

    // setup
    out = "";
    splits1 = path_split(path1);
    splits2 = path_split(path2);

    // obtain number of directory ups in path 2
    dirs_up = 0;
    for (size_t i = 0; i < splits2.size(); i++) {
        if (splits2[i] == "..") {
            dirs_up++;
        }
    }

    // drop path1 elements as path2 dir ups
    for (int i = 0; i < dirs_up; i++) {
        splits1.pop_back();
    }

    // append path1 to out
    if (path1[0] == '/') {
        out += "/";
    }
    for (size_t i = 0; i < splits1.size(); i++) {
        out += splits1[i];
        out += "/";
    }

    // append path2 to out
    for (size_t i = dirs_up; i < splits2.size(); i++) {
        out += splits2[i];
        out += "/";
    }

    // remove trailing slashes
    for (int i = out.length() - 1; i > 0; i--) {
        if (out[i] == '/') {
            out.pop_back();
        } else {
            break;
        }
    }
}

}  // end of wave namepsace
