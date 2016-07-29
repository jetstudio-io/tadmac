#include <string>
#include <sstream>
#include <vector>

const double PKG_WB_SIZE=7;
const double PKG_ACK_SIZE=11;
const double PKG_DATA_SIZE=24;

std::vector<int> &split(const std::string &s, char delim, std::vector<int> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(atoi(item.c_str()));
    }
    return elems;
}


std::vector<int> split(const std::string &s, char delim) {
    std::vector<int> elems;
    split(s, delim, elems);
    return elems;
}
