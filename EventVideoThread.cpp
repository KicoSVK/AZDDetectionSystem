#include "EventVideoThread.h"

void EventVideoThread::decreaseCounterForVideosToEncode()
{
	std::lock_guard<std::mutex> l(_mutexForCounterForVideosToEncode);

	auto i = std::begin(counterForVideosToEncode);
	int test = 0;
	while (i != std::end(counterForVideosToEncode)) {		
		(*i).TTL--;		
		if ((*i).TTL < 1)
		{
			//addToVideosToEncode((*i));
			videosToEncodeQueue.push((*i));
			i = counterForVideosToEncode.erase(i);
		}
		else
			++i;
	}
}

void EventVideoThread::addToCounterForVideosToEncode(VideoToEncode videoToEncode)
{
	std::lock_guard<std::mutex> l(_mutexForCounterForVideosToEncode);
	counterForVideosToEncode.push_back(videoToEncode);
}

std::thread EventVideoThread::runVideoEncode()
{
	return std::thread(&EventVideoThread::videoEncode, this);
}

int EventVideoThread::getNumOfVideosInCounter()
{
	std::lock_guard<std::mutex> l(_mutexForCounterForVideosToEncode);
	return counterForVideosToEncode.size();
}

void EventVideoThread::videoEncode() {
	while (!*stopGrabFrames)
	{
		try
		{
			if (videosToEncodeQueue.size() < 1)
			{
				sleep(100);				
			}
			else
			{

				VideoToEncode videoToEncode = videosToEncodeQueue.pop();


				std::string originalPath = videoToEncode.filePathName.substr(0, videoToEncode.filePathName.find_last_of("//") + 1);
				std::string pathToVideos = "/mnt/nvme/" + originalPath;

				createFolder(pathToVideos);

				std::string fileName = videoToEncode.filePathName.substr(videoToEncode.filePathName.find_last_of("//") + 1);
				std::string beforeUploadName = "v" + fileName;
				std::string afterUploadName = fileName;

				std::vector<cv::Mat> bufferedImagesToWrite = getBufferedImages();

				cv::Mat actualFrame = bufferedImagesToWrite.front();

				if (actualFrame.rows > 0 && actualFrame.cols > 0 && actualFrame.channels() == 3)
				{
					bufferedImagesToWrite.erase(bufferedImagesToWrite.begin());
					cv::VideoWriter actualEventVideoWriter = createJetsonVideoWriter(pathToVideos, beforeUploadName, actualFrame.cols, actualFrame.rows);
					actualEventVideoWriter.write(actualFrame);

					for (auto& bufferedImageToWrite : bufferedImagesToWrite) // access by reference to avoid copying
					{
						if (bufferedImageToWrite.rows > 0 && bufferedImageToWrite.cols > 0 && bufferedImageToWrite.channels() == 3)
							actualEventVideoWriter.write(bufferedImageToWrite);
						else
						{
							std::cout << "Error in video encoding!" << std::endl;
							std::cout << __LINE__ << " " << __func__ << std::endl;

							std::ofstream fileToWrite;
							fileToWrite.open("exceptions.log");
							fileToWrite << "Error in video encoding!" << "'\n";
							fileToWrite << __LINE__ << " " << __func__ << std::endl << std::endl;
							fileToWrite.close();
						}
					}
					actualEventVideoWriter.release();
					rename(std::string(pathToVideos + beforeUploadName + ".mp4").c_str(), std::string(pathToVideos + afterUploadName + ".mp4").c_str());
				}
				else
				{
					std::cout << "Error in video encoding!" << std::endl;
					std::cout << __LINE__ << " " << __func__ << std::endl;

					std::ofstream fileToWrite;
					fileToWrite.open("exceptions.log");
					fileToWrite << "Error in video encoding!" << "'\n";
					fileToWrite << __LINE__ << " " << __func__ << std::endl << std::endl;
					fileToWrite.close();
				}				
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
	std::cout << "Quiting runVideoEncode" << std::endl;
}

void EventVideoThread::createFolder(std::string folderName)
{
#if __has_include("cameraGrabModule.h")    
	struct stat st = { 0 };
	if (stat(folderName.c_str(), &st) == -1) {
		mkdir(folderName.c_str(), 0777);
	}
#endif
}

cv::VideoWriter EventVideoThread::createWindowsVideoWriter(std::string folderForVideoRecorder, int videoWidth, int videoHeight)
{
	cv::VideoWriter newVideoWriter;
	const auto p1 = std::chrono::system_clock::now();
	int timeStamp = std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count();	
	std::string recordPath = folderForVideoRecorder + std::to_string(timeStamp) + ".avi";	

	return newVideoWriter;
}


cv::VideoWriter EventVideoThread::createJetsonVideoWriter(std::string folderForVideoRecorder, int videoWidth, int videoHeight, double nsSplit)
{
	cv::VideoWriter newVideoWriter;
	const auto p1 = std::chrono::system_clock::now();
	int timeStamp = std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count();
	//std::string recordPath = "/mnt/nvme/record/areas/time" + std::to_string(timeStamp) + ".mp4";
	std::string recordPath = folderForVideoRecorder + std::to_string(timeStamp);
	std::string pipeline = "";

	std::stringstream nsSplitStream;
	nsSplitStream << std::fixed << std::setprecision(0) << nsSplit;
	std::string nsSplitString = nsSplitStream.str();

	if (nsSplit > 0)
	{
		pipeline = "appsrc ! videoconvert ! omxh264enc ! h264parse ! queue ! splitmuxsink location=" + recordPath + "%02d.mp4 max-size-time=" + nsSplitString;

	}
	else
	{
		pipeline = "appsrc ! videoconvert ! omxh264enc ! h264parse ! queue ! splitmuxsink location=" + recordPath + ".mp4";
	}

	newVideoWriter.open(pipeline, cv::CAP_GSTREAMER, (double)*cameraFps, cv::Size(videoWidth, videoHeight), true);
	return newVideoWriter;
}

cv::VideoWriter EventVideoThread::createJetsonVideoWriter(std::string folderForVideoRecorder, std::string videoName, int videoWidth, int videoHeight, double nsSplit)
{
	cv::VideoWriter newVideoWriter;
	std::string recordPath = folderForVideoRecorder + videoName;
	std::string pipeline = "";

	std::stringstream nsSplitStream;
	nsSplitStream << std::fixed << std::setprecision(0) << nsSplit;
	std::string nsSplitString = nsSplitStream.str();


	if (nsSplit > 0)
	{
		pipeline = "appsrc ! videoconvert ! omxh264enc ! h264parse ! queue ! splitmuxsink location=" + recordPath + "%02d.mp4 max-size-time=" + nsSplitString;
	}
	else
	{
		pipeline = "appsrc ! videoconvert ! omxh264enc ! h264parse ! queue ! splitmuxsink location=" + recordPath + ".mp4";
	}
	newVideoWriter.open(pipeline, cv::CAP_GSTREAMER, (double)*cameraFps, cv::Size(videoWidth, videoHeight), true);
	return newVideoWriter;
}

void EventVideoThread::bufferImage(cv::Mat frame)
{
	std::lock_guard<std::mutex> l(_mutexForBufferedImages);
	bufferedImages.push_back(frame);	
	if (bufferedImages.size() > *eventVideoDuration)
	{
		bufferedImages.erase(bufferedImages.begin());		
	}
	if (bufferReady == false)
	{
		if (bufferedImages.size() > *cameraFps)
			bufferReady = true;
	}
}

std::vector<cv::Mat> EventVideoThread::getBufferedImages()
{
	std::lock_guard<std::mutex> l(_mutexForBufferedImages);
	return bufferedImages;
}