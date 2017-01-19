#include "slam/utils/utils.hpp"


namespace slam {


void rmtrailslash(std::string &path)
{
    if (path.length() > 0) {
        std::string::iterator it = path.end() - 1;

        if (*it == '/') {
            path.erase(it);
        }
    }
}

} // end of slam namespace
