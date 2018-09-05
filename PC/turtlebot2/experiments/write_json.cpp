#include <jsoncpp/json/writer.h>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>

using namespace std;

int main()
{
    /*std::ofstream fid;
    fid.open("test.json");
    Json::Value fromScratch;
    Json::Value array;
    array.append("hello");
    array.append("world");
    fromScratch["hello"] = "world";
    fromScratch["number"] = 2;
    fromScratch["array"] = array;
    fromScratch["object"]["hello"] = "world";

    Json::StyledWriter styledWriter;
    //std::cout << styledWriter.write(fromScratch);
    fid << styledWriter.write(fromScratch);
    fid.close();*/

    /////
    std::ofstream fid;
    fid.open("test.json");
    Json::Value array;
    Json::Value v1;
    v1["timestamp"] = 123;
    v1["position_x"] = 0.0;
    v1["position_y"] = 0.0;
    v1["bool"] = !true;
    array.append(v1);
    //Json::StyledWriter styledWriter;
    //fid << styledWriter.write(v1);

    Json::Value v2;
    v2["timestamp"] = 124;
    v2["position_x"] = 0.1;
    v2["position_y"] = 0.1;
    v2["bool"] = false;
    array.append(v2);
    //fid << styledWriter.write(v2);
    Json::StyledWriter styledWriter;
    fid << styledWriter.write(array);
    fid.close();

    return 0;
}
