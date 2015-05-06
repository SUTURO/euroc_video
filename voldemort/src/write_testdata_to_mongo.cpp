// zum Compilen: g++ tutorial.cpp -pthread -lmongoclient -lboost_thread-mt -lboost_system -lboost_regex -ljsoncpp -lboost_filesystem -std=gnu++0x -o tutorial
#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include <string>
#include <thread>
#include <iostream>
#include <jsoncpp/json/json.h>


using namespace mongo;

DBClientConnection *mongodb_conn;
std::string collection;

void write_json_to_mongo (std::string json_message, std::string collection_name) {
        json_message.erase(remove_if(json_message.begin(), json_message.end(), ::isspace), json_message.end());

        std::string mongodb = "localhost";
        collection = "robocop_test_results."+collection_name;

        BSONObj doc = fromjson(json_message);

        std::string errmsg;
        mongodb_conn = new DBClientConnection(true);
        if (!mongodb_conn->connect(mongodb, errmsg)) {
                cout << "Can't connect to mongodb!" << endl;
        } else {
                mongodb_conn->insert(collection, doc);
                delete mongodb_conn;
        }
}

int main(int argc, char **argv) {
        // Dummy Data
        // Thanks to: http://www.thomaswhitton.com/blog/2013/06/28/json-c-plus-plus-examples/
        Json::Value jv;
        Json::Value array;
        array.append("hello");
        array.append("world");
        jv["hello"] = "world";
        jv["number"] = 2;
        jv["array"] = array;
        jv["object"]["hello"] = "world";

        Json::FastWriter fastWriter;
        std::string json_message = fastWriter.write(jv);
        
        // Dummy "random" collection name, unix time stamp
        std::time_t result = std::time(nullptr);
        std::string time = std::asctime(std::localtime(&result));
        time.erase(remove_if(time.begin(), time.end(), ::isspace), time.end());
        write_json_to_mongo(json_message, time);
}
