#include "systemThreads.h"
#define S(x) #x
#define S_(x) S(x)
#define S__LINE__ S_(__LINE__)

void runNetForward(threadQueue<OutputFrame>& dnnQueue, threadQueue<OutputFrame>& queueFromCamera, cv::dnn::Net& netDnn, int JETSON, cv::VideoCapture& cap, bool& stopGrabFrames){
	OutputFrame outputFrameFirst;
	cv::cuda::GpuMat frame_u;
	InferenceTimer inferenceTimerBlob;
	InferenceTimer inferenceTimerForward;

	if (!JETSON)
	{
		cap >> outputFrameFirst.frameOriginal;

		frame_u.upload(outputFrameFirst.frameOriginal);
		cv::cuda::resize(frame_u, frame_u, cv::Size(1024, 0.8366013071895425 * 1024));
		frame_u.download(outputFrameFirst.frameForDisplay);

		outputFrameFirst.frameForProcessing = outputFrameFirst.frameForDisplay.clone();

		frame_u.upload(outputFrameFirst.frameOriginal);
		cv::cuda::resize(frame_u, frame_u, cv::Size(inpWidth, inpHeight));
		frame_u.download(outputFrameFirst.frameForDnn);

		queueFromCamera.push(outputFrameFirst);
	/*	if (queueFromCamera.push(outputFrame) == 1)
			delete outputFrame;*/
	}
		
	while (!stopGrabFrames)
	{
		OutputFrame outputFrame;
		try
		{
			if (JETSON)
				outputFrame = queueFromCamera.pop();
			else
			{
				cap >> outputFrame.frameOriginal;
				// kiac kiac, pre testovanie na windowse
				//cap >> outputFrame.frameOriginal;

				frame_u.upload(outputFrame.frameOriginal);
				cv::cuda::resize(frame_u, frame_u, cv::Size(1024, 0.8366013071895425 * 1024));
				frame_u.download(outputFrame.frameForDisplay);

				outputFrame.frameForProcessing = outputFrame.frameForDisplay.clone();

				frame_u.upload(outputFrame.frameOriginal);
				cv::cuda::resize(frame_u, frame_u, cv::Size(inpWidth, inpHeight));
				frame_u.download(outputFrame.frameForDnn);
			}			

			if (showPartialFps)
				inferenceTimerBlob.timerInitialization();

			vector<cv::Point> noDetectingAreaTemp;

			if (noDetectingArea.size() > 0) {
				for (int i = 0; i < noDetectingArea.size(); i++)
				{
					noDetectingAreaTemp.push_back(cv::Point2f(((double)inpWidth / outputFrame.frameForDisplay.cols) * (noDetectingArea.at(i).x), ((double)inpHeight / outputFrame.frameForDisplay.rows) * (noDetectingArea.at(i).y)));				
				}
			}

			if (noDetectingArea.size() > 0) {
				cv::fillPoly(outputFrame.frameForDnn, noDetectingAreaTemp, cv::Scalar(0, 0, 0));
			}

			cv::Mat blob;
			std::vector<cv::Mat> outs;
			//cv::Mat smallerFrame;
			//smallerFrame = outputFrame->frame.clone();

			/*if (JETSON)
			{*/
			/*frame_u.upload(smallerFrame);
			cv::cuda::resize(frame_u, frame_u, cv::Size(inpWidth, inpHeight));
			frame_u.download(smallerFrame);*/
			cv::dnn::blobFromImage(outputFrame.frameForDnn, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0, 0, 0), true, false);
			//}
			/*else {
				cv::dnn::blobFromImage(outputFrame->frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0, 0, 0), true, false);
			}*/

			//Sets the input to the network
			netDnn.setInput(blob);
			if (showPartialFps)
				std::cout << "Average blob time: " << inferenceTimerBlob.getInferenceAverageTimeInMilliseconds(true) << " ms FPS: " << inferenceTimerBlob.getAverageFps() << std::endl;
			//std::cout << "Blob FPS: " << inferenceTimerBlob.getAverageFps() << "time: " << inferenceTimerBlob.getInferenceTimeInMilliseconds() << " ms" << std::endl;

			if (showPartialFps)
				inferenceTimerForward.timerInitialization();

			netDnn.forward(outs, getOutputsNames(netDnn));
			//std::vector<cv::Mat> the_copy(outs);
			//outputFrame->outs = the_copy;
			std::vector<cv::Mat> the_copy;

			for (int i = 0; i < outs.size(); i++) {
				cv::Mat novy;
				novy = outs[i].clone();
				the_copy.push_back(novy);
			}

			outputFrame.outs = the_copy;

			if (showPartialFps)
				std::cout << "Average forward time: " << inferenceTimerForward.getInferenceAverageTimeInMilliseconds(true) << " ms FPS: " << inferenceTimerForward.getAverageFps() << std::endl;

			dnnQueue.push(outputFrame);

			//if (dnnQueue.push(outputFrame) == 1)
			//	delete outputFrame;

			//postprocess(outputFrame->frame, outputFrame->outs);
			//jetsonImShow("test", outputFrame->frame);
			//cv::waitKey();
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
	std::cout << "Quiting runNetForward" << std::endl;
}


void runPostProcess(threadQueue<OutputFrame>& dnnQueue, threadQueue<OutputFrame>& fromPostProcessQueue, int& cameraFps, bool& stopGrabFrames, bool& showPartialFps, std::vector<std::string>& classes, bool& isDebugMode) {
	//OutputFrame* outputFrame = NULL;
	InferenceTimer inferenceTimerPostProcess;
	std::vector<DetectedObject> detectedObjectsInScene;

	std::vector<int> detectedIndexes;

	while (!stopGrabFrames)
	{
		OutputFrame outputFrame;
		try {
			outputFrame = dnnQueue.pop();
			if (showPartialFps)
				inferenceTimerPostProcess.timerInitialization();

			// Processing (copy) all detected objects in scene yet
			//outputFrame->detectedObjectsInScene = detectedObjectsInScene;
			for (int i = 0; i < detectedObjectsInScene.size(); i++)
			{
				outputFrame.detectedObjectsInScene.push_back(detectedObjectsInScene.at(i));
			}

			// Processing (copy) all detected object indexes yet
			for (int i = 0; i < detectedIndexes.size(); i++)
			{
				outputFrame.detectedIndexes.push_back(detectedIndexes.at(i));
			}

			postprocess(outputFrame, cameraFps, outputFrame.outs, classes, confThreshold, nmsThreshold, isDebugMode);

			// Clear all old detected yet
			detectedObjectsInScene.clear();

			// Clear all old indexes
			detectedIndexes.clear();

			// And save all new detected objects to vector
			//detectedObjectsInScene = outputFrame->detectedObjectsInScene;
			for (int i = 0; i < outputFrame.detectedObjectsInScene.size(); i++)
			{
				detectedObjectsInScene.push_back(outputFrame.detectedObjectsInScene.at(i));
			}

			// And save all new detected object indexes to vector
			for (int i = 0; i < outputFrame.detectedIndexes.size(); i++)
			{
				detectedIndexes.push_back(outputFrame.detectedIndexes.at(i));
			}

			fromPostProcessQueue.push(outputFrame);

			//if (fromPostProcessQueue.push(outputFrame) == 1)
			//	delete outputFrame;

			if (showPartialFps)
				std::cout << "PostProcess time: " << inferenceTimerPostProcess.getInferenceAverageTimeInMilliseconds(true) << " ms" << " FPS: " << inferenceTimerPostProcess.getAverageFps() << std::endl;
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
	std::cout << "Quiting runPostProcess" << std::endl;
}


void runDraw(threadQueue<OutputFrame>& fromPostProcessQueue, EventVideoThread& eventVideoThread, int JETSON, bool& showFrames, bool& stopGrabFrames, bool& showGlobalFps, bool& isDebugMode, bool& isShowInferenceTimeAndFps, cv::VideoCapture& cap, cv::VideoWriter& videoWriterWithAnnotations, cv::VideoWriter& videoWriterOriginal) {
	
	// Create a window    
	if (showFrames)
	{
		cv::namedWindow(kWinName, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL | cv::WINDOW_KEEPRATIO);
		cv::setMouseCallback(kWinName, mouseEvent, NULL);
		//cv::resizeWindow(kWinName, 1024, 1024);
		cv::resizeWindow(kWinName, 1024, 0.8366013071895425 * 1024);
	}
	
	// Mouse function call back
	//if (showFrames)
	//    cv::setMouseCallback(kWinName, mouseEventForCarMoving, NULL);
	//OutputFrame* outputFrame = NULL;
	OutputFrame outputFrame;

	// Inference time measurement
	InferenceTimer inferenceGlobalTimer;
	InferenceTimer inferenceDrawTimer;
	
	cv::Mat frame;
	std::vector<cv::Mat> outs;
	//std::vector<DetectedObject> detectedObjectsInScene;
	// Vector for storing contour
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	//sleep(5000);
	// Frame count
	int frameCount = 0;
	int showFramesHeight = 1024;
	
	cv::cuda::GpuMat writeFrameResize;

	std::vector<std::string> linesProceeded{};
	
	while (!stopGrabFrames)
	{
		try
		{
			//linesProceeded.clear();
			/*std::cout << "fromPostProcessQueue" << fromPostProcessQueue.size() << std::endl;*/
			//std::cout << "new frame" << std::endl;

			//while (fromPostProcessQueue.size() < 1)
			//{
			//	sleep(100);
			//}
			if (showGlobalFps)
				inferenceGlobalTimer.timerInitialization();

			if (showGlobalFps)
				inferenceDrawTimer.timerInitialization();

			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

			outputFrame = fromPostProcessQueue.pop();
			frame = outputFrame.frameForDisplay;
			outs = outputFrame.outs;
			//detectedObjectsInScene = outputFrame->detectedObjectsInScene;

			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

			if (showGlobalFps)
				inferenceDrawTimer.timerInitialization();

			//frameResizeRationX = frame.cols / ((frame.cols / frame.rows) * (double)showFramesHeight);
			//frameResizeRationY = frame.rows / (double)showFramesHeight;
			//std::cout << frameResizeRationX << "/" << frameResizeRationY << std::endl;

			//resize(frame, frame, cv::Size(int((frame.cols / frame.rows)) * showFramesHeight, showFramesHeight), 0.0, 0.0, cv::INTER_AREA); //319 - 19 ms

			//writeFrameResize.upload(frame); //16 ms
			//cv::cuda::resize(writeFrameResize, writeFrameResize, cv::Size(int((frame.cols / frame.rows)) * showFramesHeight, showFramesHeight));
			//writeFrameResize.download(frame);

			//auto utcTimeStamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			//std::stringstream utcTimeStampStringStream;
			//utcTimeStampStringStream << utcTimeStamp;
			//std::string pathToImage = std::string("/mnt/nvme/imagesToUpload/") + utcTimeStampStringStream.str();
			//cv::imwrite(pathToImage + ".jpg", frame);
			//evenTypeHandler.sendVehiclePassage(std::string("test").c_str());

			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

		

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

		//if (isCarDisplayed)
		//	drawCar(frame);

		// check if monitored area contains any obstacle
		//checkObstacleInMonitoredArea(outputFrame->frameForDisplay, outputFrame->detectedObjectsInScene);

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

		// check object movement in traffic area
		//checkObjectMovementInTrafficArea(frame, outputFrame->detectedObjectsInScene);

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

		// check traffic light state
		trafficLightCheck(outputFrame);

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
		// check enter to monitored area
		monitoredAreaCheck(frame, outputFrame.detectedObjectsInScene);

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

		//checkUnpermittedStoppingInMonitoredArea(frame, outputFrame->detectedObjectsInScene);

		//checkUnpermittedStoppingInMonitoredArea(frame, outputFrame->detectedObjectsInScene);

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

		// draw control line
		// kiac kiac
		//checkAndDrawControlLine(frame, outputFrame.detectedObjectsInScene, outputFrame.detectedIndexes);

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
		checkIfBusIsLocatedInFrame(frame, outputFrame.detectedObjectsInScene);

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

		// draw monitored area
		drawMonitoredArea(frame);

		drawUnpermittedVehiclePassageArea(frame);

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
		// draw access control area
		//drawAccessControlArea(frame);

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
		// draw traffic area
		drawAllTrafficAreas(frame); //6ms

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

		// keyboard keypress manager
		// Get key press
		keypressController(frame, cv::waitKey(1));

		//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

			// Stop the program if reached end of video
			if (frame.empty()) {
				std::cout << "Video processing is done !" << std::endl;
				//std::cout << "Output file is stored as " << outputFile << std::endl;
				cv::waitKey(3000);
				break;
			}
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));

			// Create a 4D blob from a frame.
			//cv::dnn::blobFromImage(frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0, 0, 0), true, false);
			//cv::dnn::blobFromImage(frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0, 0, 0), true, false);       

			//Sets the input to the network
			//net.setInput(blob);

			// Runs the forward pass to get output of the output layers
			//cout << endl;

			// Write the frame with the detection boxes

			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			//if (saveAPicture) {
			//	std::string nameOfFrame = "img/frame" + std::to_string(frameCount) + ".jpg";
			//	imwrite(nameOfFrame, frame);
			//	frameCount++;
			//}

			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			if (isDisplayedMainMenu) {
				int spaceBettewnLines = 15;
				cv::putText(frame, "Application menu:", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				cv::putText(frame, "- for define monitored area, press Y/Z", cv::Point(20, 75), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				cv::putText(frame, "- for define traffic border, press X", cv::Point(20, 110), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				cv::putText(frame, "- for define traffic light area, press A", cv::Point(20, 145), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				cv::putText(frame, "- for define access control area, press V", cv::Point(20, 180), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				cv::putText(frame, "- for define no detecting area, press K", cv::Point(20, 215), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				cv::putText(frame, "- for play/pause video, press P", cv::Point(20, 250), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				cv::putText(frame, "- for enable debug mode, press D", cv::Point(20, 285), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				//cv::putText(frame, "- for record a video, press S", cv::Point(20, 285), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				//cv::putText(frame, "- for show inference time and FPS, press F", cv::Point(20, 320), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
				//cv::putText(frame, "- for save configuration, press S", cv::Point(10, 85), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
				//cv::putText(frame, "- for load configuration, press L", cv::Point(10, 105), cv::FONT_HERSHEY_DUPLEX, 0.4, CV_RGB(255, 255, 255), 1.6);
				cv::putText(frame, "- for exit from menu, press ESC", cv::Point(20, 365), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, CV_RGB(255, 255, 255), 1.6, cv::LINE_AA);
			}
			else {
				// main info message in the bottom of window
				//cv::putText(frame, "Press TAB or M for insert to menu", cv::Point(10, frame.rows - 10), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.2);
				//cv::putText(frame, "Press TAB or M for insert to menu", cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.6, CV_RGB(255, 255, 255), 1.2);
				cv::putText(frame, "Press TAB or M for insert to menu", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1.0 * scaleFactor, cv::Scalar(255, 255, 255), 1.6, cv::LINE_AA);
			}
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			while (isPlaying == false) {
				if (cv::waitKey()) {
					isPlaying = true;
					break;
				}
			}

			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			if (isShowInferenceTimeAndFps == true) {
				// Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
				//std::vector<double> layersTimes;
				//double freq = cv::getTickFrequency() / 1000;
				//double t = net.getPerfProfile(layersTimes) / freq;
				//double dnnFps = 1 / (t / 1000);
				//std::string label = cv::format("DNN inference time: %.2f ms / %.2f FPS", t, dnnFps);

				// Display inference time in frame
				/* putText(frame, label, cv::Point(frame.cols - (frame.cols * 0.38), 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255));*/

				std::string text = cv::format("Total inference time: %.2f ms / %.2f FPS", 1000.00, 100.00);
				int fontFace = cv::FONT_HERSHEY_SIMPLEX;
				double fontScale = 0.5;
				int thickness = 1;
				int baseline = 0;

				cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);

				if (fontScale > 1.0)
					thickness = int(round(fontScale));

				// center the text
				textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
				cv::Point textOrg(frame.cols - textSize.width, textSize.height + 10);

				std::string label = cv::format("Total inference time: %.2f ms / %.2f FPS", inferenceGlobalTimer.getInferenceAverageTimeInMilliseconds(false), inferenceGlobalTimer.getAverageFps());
				//putText(frame, label, textOrg, fontFace, fontScale, cv::Scalar(255, 0, 255), thickness, 8);
				putText(frame, label, textOrg, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(255, 255, 255), thickness, 8);
			}

			//std::cout << "__LINE__" << std::string(S__LINE__) + " " + std::string(__func__) << std::endl;
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			if (showGlobalFps) {

				std::string text = cv::format("%.2f ms / %.2f FPS", 40.00, 50.00);
				int fontFace = cv::FONT_HERSHEY_SIMPLEX;
				double fontScale = 1;
				int thickness = 1;
				int baseline = 0;

				cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
				baseline += thickness;
				fontScale = (frame.cols / 2) / textSize.width;

				if (fontScale > 1.0)
					thickness = int(round(fontScale));

				// center the text
				textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
				cv::Point textOrg(frame.cols - textSize.width, textSize.height + 10);

				//// draw the box
				//rectangle(img, textOrg + Point(0, baseline),
				//    textOrg + Point(textSize.width, -textSize.height),
				//    Scalar(0, 0, 255));
				//// ... and the baseline first
				//line(img, textOrg + Point(0, thickness),
				//    textOrg + Point(textSize.width, thickness),
				//    Scalar(0, 0, 255));

				// then put the text itself
				text = cv::format("%.2f ms / %.2f FPS", inferenceGlobalTimer.getInferenceTimeInMilliseconds(), inferenceGlobalTimer.getFps());
				putText(frame, text, textOrg, fontFace, fontScale, cv::Scalar(255, 0, 255), thickness, 8);


			}
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			// Display main frame window		
			if (showFrames)
			{
				if (showFramesResize) {
					cv::Mat showFrame;
					//resize(frame, showFrame, cv::Size(int((frame.cols / frame.rows)) * showFramesHeight, showFramesHeight), 0.0, 0.0, cv::INTER_AREA);
					jetsonImShow(kWinName, frame);
				}
				else
				{
					jetsonImShow(kWinName, frame); //11ms
				}
			}
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			if (bufferAndSendVideo)
			{
				eventVideoThread.bufferImage(frame);
				eventVideoThread.decreaseCounterForVideosToEncode();				
			}
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			if (writeVideoWithAnnotations)
			{
				//writeFrameResize.upload(frame); //16 ms
				//cv::cuda::resize(writeFrameResize, writeFrameResize, cv::Size(int((frame.cols / frame.rows)) * showFramesHeight, showFramesHeight));
				//writeFrameResize.download(frame);
				try {
					videoWriterWithAnnotations.write(frame); //33ms
				}
				catch (const std::exception& e) // caught by reference to base
				{
					std::cout << "Can't write frame to videoWriterWithAnnotations!" << std::endl;
					std::cout << "Exception was caught. Message: '" << e.what() << std::endl;
					std::cout << __LINE__ << " " << __func__ << std::endl;

					std::ofstream fileToWrite;
					fileToWrite.open("exceptions.log");
					fileToWrite << "Exception was caught. Message: '" << e.what() << "'\n";
					fileToWrite << __LINE__ << " " << __func__ << std::endl << std::endl;
					fileToWrite.close();

					//// Print Strings stored in Vector
					//for (int i = 0; i < linesProceeded.size(); i++)
					//	std::cout << linesProceeded[i] << "\n";
				}
				
			}
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			if (writeVideoOriginal)
			{
				//writeFrameResize.upload(frame); //16 ms
				//cv::cuda::resize(writeFrameResize, writeFrameResize, cv::Size(int((frame.cols / frame.rows)) * showFramesHeight, showFramesHeight));
				//writeFrameResize.download(frame);
				try {
					videoWriterOriginal.write(outputFrame.frameOriginal); //33ms
				}				
				catch (const std::exception& e) // caught by reference to base
				{
					std::cout << "Can't write frame to videoWriterOriginal!" << std::endl;					
					std::cout << "Exception was caught. Message: '" << e.what() << std::endl;
					std::cout << __LINE__ << " " << __func__ << std::endl;

					std::ofstream fileToWrite;
					fileToWrite.open("exceptions.log");
					fileToWrite << "Exception was caught. Message: '" << e.what() << "'\n";
					fileToWrite << __LINE__ << " " << __func__ << std::endl << std::endl;
					fileToWrite.close();


					//// Print Strings stored in Vector
					//for (int i = 0; i < linesProceeded.size(); i++)
					//	std::cout << linesProceeded[i] << "\n";
				}
			}
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			//delete outputFrame;
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			if (showGlobalFps) {
				std::cout << "Average (draw) time: " << inferenceDrawTimer.getInferenceAverageTimeInMilliseconds(true) << " ms FPS: " << inferenceDrawTimer.getAverageFps() << std::endl; //25ms
			}
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
			if (showGlobalFps) {
				//std::cout << "Global FPS: " << inferenceTimer.getFps() << " time: " << inferenceTimer.getInferenceTimeInMilliseconds() << std::endl;
				std::cout << "Global time: " << inferenceGlobalTimer.getInferenceAverageTimeInMilliseconds(true) << " ms FPS: " << inferenceGlobalTimer.getAverageFps() << std::endl; //25ms
				// inferenceGlobalTimer.getInferenceAverageTimeInMilliseconds(true);
			}
			//linesProceeded.push_back(std::string(S__LINE__) + std::string(" " + std::string(__func__)));
		}
		catch (const std::exception& e) // caught by reference to base
		{
			std::cout << " a standard exception was caught, with message '"
				<< e.what() << "'\n";
			std::cout << __LINE__ << " " << __func__ << std::endl;		
			

			//// Print Strings stored in Vector
			//for (int i = 0; i < linesProceeded.size(); i++)
			//	std::cout << linesProceeded[i] << "\n";
		}
	}

	std::cout << "Quiting runDraw" << std::endl;	
}

void keypressController(cv::Mat frame, int key) {
/*
legend:
	key tab (show menu)	

	key k (no detecing area)	
	key j (camera calibration)
	key u (add control line)			
	key a (add traffic light area)
	key z/y (add monitored area)
	key v (add unpermitted passage area)
	key x (add traffic area)

	key d (allow debug mode)
	key p (play/pause)
	key s (save configuration)
	key g (add testing car)	
	
	key c (clear)
	key ESC (exit menus)
	key q (quit and save recorded video)
*/

	// key tab (show menu)
	if (key == 9 || key == 109) {
		//writeEventVideo();
		if (isDisplayedMainMenu == true)
			isDisplayedMainMenu = false;
		else
			isDisplayedMainMenu = true;
	}
	// key k (no detecing area)
	if (key == 107 || key == 75) {
		isDisplayedMainMenu = false;

		if (isDisplayedNoDetectingAreaScenario == true)
			isDisplayedNoDetectingAreaScenario = false;
		else
			isDisplayedNoDetectingAreaScenario = true;

		addNoDetectingArea(frame);
	}

	// key u (add control line)
	else if (key == 117 || key == 85) {
		if (isDisplayedControlLineScenario == true)
			isDisplayedControlLineScenario = false;
		else
			isDisplayedControlLineScenario = true;

		addControlLine(frame);
	}
	// key g (add testing car)
	else if (key == 103 || key == 71) {
		if (isCarDisplayed == true) {
			isCarDisplayed = false;
		}
		else {
			isCarDisplayed = true;
		}
	}
	// key b
	//else if (key == 98) {
	//	if (detectCar == true)
	//		detectCar = false;
	//	else
	//		detectCar = true;
	//}
	// key r (reset detections)
	//else if (key == 114) {
	//	outputFrame->detectedObjectsInScene.clear();
	//}
	// key f (show FPS)
	//if (key == 102) {
	//	if (isShowInferenceTimeAndFps == true)
	//		isShowInferenceTimeAndFps = false;
	//	else
	//		isShowInferenceTimeAndFps = true;
	//}
	// key d (allow debug mode)
	if (key == 100 || key == 68) {
		if (isDebugMode == true)
			isDebugMode = false;
		else
			isDebugMode = true;
	}
	// key s (save screenshot)
	//else if (key == 115) {
	//	if (saveAPicture == false) {
	//		saveAPicture = true;
	//	}
	//	else {
	//		saveAPicture = false;
	//	}
	//}

	// key p (play/pause)
	else if (key == 112 || key == 80) {
		if (isPlaying == false) {
			isPlaying = true;
		}
		else {
			isPlaying = false;
		}
	}
	// key s (save configuration)
	else if (key == 115 || key == 83) {
		saveConfiguration();
	}
	// key l (load configuration)
	//else if (key == 108 || key == 76) {
	//	//loadConfiguration();
	//}
	// key a (add traffic light area)
	else if (key == 97 || key == 65) {
		isDisplayedMainMenu = false;

		if (isDisplayedTrafficLightAreaScenario == true) {
			isDisplayedTrafficLightAreaScenario = false;
		}
		else {
			isDisplayedTrafficLightAreaScenario = true;
		}

		addTrafficLightArea(frame);
	}
	// key z/y (add monitored area)
	else if (key == 122 || key == 121) {
		isDisplayedMainMenu = false;
		isDisplayedTrafficAreaScenario = false;

		if (isDisplayedMonitoredAreaScenario == true)
			isDisplayedMonitoredAreaScenario = false;
		else
			isDisplayedMonitoredAreaScenario = true;

		addMonitoredArea(frame);
	}
	// key v (add unpermitted passage area)
	else if (key == 118 || key == 86) {
		isDisplayedMainMenu = false;
		isDisplayedTrafficAreaScenario = false;
		isDisplayedMonitoredAreaScenario = false;

		if (isDisplayedUnpermittedVehiclePassageScenario == true)
			isDisplayedUnpermittedVehiclePassageScenario = false;
		else
			isDisplayedUnpermittedVehiclePassageScenario = true;

		addUnpermittedVehiclePassage(frame);
	}
	// key x (add traffic area)
	else if (key == 120 || key == 88) {
		isDisplayedMainMenu = false;
		isDisplayedMonitoredAreaScenario = false;

		if (isDisplayedTrafficAreaScenario == true)
			isDisplayedTrafficAreaScenario = false;
		else
			isDisplayedTrafficAreaScenario = true;

		addTrafficArea(frame);
	}
	// key c
	else if (key == 99 || key == 67) {
		if (isDisplayedMonitoredAreaScenario) {
			trafficArea[indexOfActiveTrafficArea].getTrafficArea().clear();
		}

		if (isDisplayedTrafficAreaScenario) {
			trafficArea[indexOfActiveTrafficArea].getTrafficArea().clear();
		}
	}
	// key ESC
	else if (key == 27) {
		isDisplayedMainMenu = false;
		isDisplayedMonitoredAreaScenario = false;
		isDisplayedTrafficAreaScenario = false;
	}
	//key q (quit and save recorde video)
	else if (key == 113 || key == 81) {
		onExit(1, stopGrabFrames, cap, videoWriterWithAnnotations, videoWriterOriginal, net);
		//break;
		//return 0;
	}
}