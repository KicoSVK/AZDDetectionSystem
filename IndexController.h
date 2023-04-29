#ifndef	INDEXCONTROLLER_H_
#define INDEXCONTROLLER_H_

#include <vector> 

/*! IndexController class control detected object indexes*/
class IndexController {

private:
	std::vector<int> indexes; /*!< vector for store all existed indexes */
	int maxIndexValue = 100; /*!< max index value for detected objects indexes */

public:
	/**
	* Default constructor for IndexController class
	*/
	IndexController();

	/**
	* Default destructor for IndexController class
	*/
	~IndexController();

	/**
	* Function for return first free index from vector
	*
	* @return int free index from vector
	*/
	int getIndex();

	/**
	* Function for remove index from vector
	*
	* @param int index of detected object
	*/
	void removeIndex(int index);

	/**
	* Function for reset index controller
	*/
	void resetController();
};

#endif