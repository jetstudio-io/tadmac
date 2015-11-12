#include <string>
#include <sstream>
#include <vector>

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
