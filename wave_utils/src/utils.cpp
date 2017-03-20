#include "wave/utils/utils.hpp"


namespace wave {


void rmtrailslash(std::string &path) {
    if (path.length() > 0) {
        std::string::iterator it = path.end() - 1;

        if (*it == '/') {
            path.erase(it);
        }
    }
}

}  // end of wave namespace
