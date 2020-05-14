#include "AZDDetectionSystem.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	AZDDetectionSystem w;
	w.show();
	return a.exec();
}
