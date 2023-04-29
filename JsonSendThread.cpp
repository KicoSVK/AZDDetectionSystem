#include "JsonSendThread.h"

JsonSendThread::JsonSendThread() {

}

JsonSendThread::JsonSendThread(const char* newIpAddress, const char* newPort, const char* newCa_pem, const char* newCert_pem, const char* newKey_pem, bool* newStopGrabFrames)
{
    ipAddress = newIpAddress;
    port = newPort;
    ca_pem = newCa_pem;
    cert_pem = newCert_pem;
    key_pem = newKey_pem;
    stopGrabFrames = newStopGrabFrames;
}

JsonSendThread::~JsonSendThread()
{
}

std::thread JsonSendThread::runJsonSending()
{
    return std::thread(&JsonSendThread::jsonSending, this);
}

void JsonSendThread::jsonSending()
{
    //std::cout << __LINE__ << " " << __func__ << std::endl;
    //std::cout << "stopGrabFrames" << stopGrabFrames << __func__ << std::endl;
    while (!*stopGrabFrames)
    {

        //std::cout << __LINE__ << " " << __func__ << std::endl;
        try
        {
            if(jsonsToSendQueue.size() < 1)
            {
                sleep(100);
            }
            else
            {
                json jsonToSend = jsonsToSendQueue.pop();
                // kiac koment
                //mutualJsonClientSend(ipAddress, port, ca_pem, cert_pem, key_pem, jsonToSend);
            }
        }
        catch (const std::exception& e) // caught by reference to base
        {
            std::cout << "Exception was caught. Message: '" << e.what() << std::endl;
            std::cout << __LINE__ << " " << __func__ << std::endl;

            std::ofstream fileToWrite;
            fileToWrite.open("exceptions.log");
            fileToWrite << "Exception was caught. Message: '" << e.what() << "'\n";
            fileToWrite << __LINE__ << " " << __func__ << std::endl << std::endl;            
            fileToWrite.close();
        }
    }
    //std::cout << __LINE__ << " " << __func__ << std::endl;
}

void JsonSendThread::addJsonToSend(json jsonToSend) {
    jsonsToSendQueue.push(jsonToSend);
}


void JsonSendThread::setParams(const char* newIpAddress, const char* newPort, const char* newCa_pem, const char* newCert_pem, const char* newKey_pem, bool* newStopGrabFrames)
{
    ipAddress = newIpAddress;
    port = newPort;
    ca_pem = newCa_pem;
    cert_pem = newCert_pem;
    key_pem = newKey_pem;
    stopGrabFrames = newStopGrabFrames;
}