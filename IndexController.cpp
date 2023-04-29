#include "IndexController.h"

// Default constructor
IndexController::IndexController() {
	for (size_t i = 0; i < maxIndexValue; i++)
	{
		indexes.push_back(-1);
	}
}

// Default destructor
IndexController::~IndexController() {
	indexes.clear();
}

// Get first free index from vector
int IndexController::getIndex() {
	for (size_t i = 0; i < maxIndexValue; i++)
	{
		if (indexes.at(i) < 0) {
			indexes.at(i) = i;
			return i;
			//return indexes.at(i);
		}
	}
}

// Remove index from vector
void IndexController::removeIndex(int index) {
	for (int i = 0; i < maxIndexValue; i++)
	{
		if (indexes.at(index) == index) {
			indexes.at(index) = -1;
		}
	}	
}

void IndexController::resetController()
{
	for (size_t i = 0; i < maxIndexValue; i++)
	{
		indexes.push_back(-1);
	}
}
