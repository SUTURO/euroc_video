/*
 * call_robocop.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: andz
 */

#include <stdio.h>
#include <curl/curl.h>
#include <std_msgs/String.h>
#include <voldemort/callRobocop.h>
#include <mongo/client/dbclient.h>
#include <mongo/db/json.h>

using namespace mongo;

DBClientConnection *mongodb_conn;
CURL *curl;
CURLcode res;
std::string request = "";
std::string agent = "";
std::string collection = "";
std::string mongodb = "localhost";
bool verbose = false;

size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata) {

	std::string json_message;
	for (int c = 0; c<size*nmemb; c++) {
		json_message.push_back(ptr[c]);
	}

	std::string requestString = "request";
	std::string dataString = "data";
	json_message = "{ '" + requestString + "':'" + request + "','" + dataString + "':" + json_message + "}";

	if (verbose) {
		fprintf(stdout, "Parsing:\n############\n%s\n###########\n", json_message.c_str());
	}

	BSONObj doc;

	try {
		doc = fromjson(json_message);
	} catch (MsgAssertionException &e) {
		fprintf(stderr, "Failed parsing.\n");
		if (verbose) {
			fprintf(stderr, "Exception: %s\n", e.what());
		}
		exit(-1);
	}

	std::string errmsg;
	mongodb_conn = new DBClientConnection(true);
	if (!mongodb_conn->connect(mongodb, errmsg)) {
		fprintf(stderr, "Can't connect to mongodb!\n");
		curl_easy_cleanup(curl);
		exit(-1);
	} else {
		mongodb_conn->insert(collection, doc);
		delete mongodb_conn;
	}

	return size*nmemb;
}

int main(int argc, char **argv) {

	int c;
	while ((c = getopt(argc, argv, "r:a:c:m:v")) != -1) {
		switch (c) {
		case 'r':
			request = optarg;
			break;
		case 'a':
			 agent = optarg;
			 break;
		case 'c':
			 collection = optarg;
			 break;
		case 'm':
			 mongodb = optarg;
			 break;
		case 'v':
			verbose = true;
			break;
		default:
			printf("Usage: %s -r Request -c Collection [-m mongoDB] [-a agent] [-v (verbose)] \n", argv[0]);
			exit(-1);
		}
	}

	if (request.empty()) {
		fprintf(stderr, "No request given.\n");
		exit(-1);
	}

	if (collection.empty()) {
		fprintf(stderr, "No collection given.\n");
		exit(-1);
	}

	curl = curl_easy_init();
	if(curl) {
		curl_easy_setopt(curl, CURLOPT_URL, request.c_str());
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

		if (!agent.empty()) {
			curl_easy_setopt(curl, CURLOPT_USERAGENT, agent.c_str());
		}

		curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);

		if (verbose) {
			curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
		}

		res = curl_easy_perform(curl);

		if(res != CURLE_OK) {
			fprintf(stderr, "Failed performing request: %s\n", curl_easy_strerror(res));
			exit(-1);
		}

		curl_easy_cleanup(curl);
	}

	return 0;
}
