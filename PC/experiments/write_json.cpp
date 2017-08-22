#include <jsoncpp/json/writer.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <iostream>
#include <fstream>

int main()
{
    Json::Value fromScratch;
    Json::Value array;
    array.append("hello");
    array.append("world");
    fromScratch["hello"] = "world";
    fromScratch["number"] = 2;
    fromScratch["array"] = array;
    fromScratch["object"]["hello"] = "world";

    Json::StyledWriter styledWriter;
    std::cout << styledWriter.write(fromScratch);
    return 0;
}
