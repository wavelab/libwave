#include "wavelib/utils/utils.hpp"


namespace wavelib {


void rmtrailslash(std::string &path)
{
    if (path.length() > 0) {
        std::string::iterator it = path.end() - 1;

        if (*it == '/') {
            path.erase(it);
        }
    }
}

} // end of wavelib namespace
