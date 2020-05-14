#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_AZDDetectionSystem.h"

class AZDDetectionSystem : public QMainWindow
{
	Q_OBJECT

public:
	AZDDetectionSystem(QWidget *parent = Q_NULLPTR);

private:
	Ui::AZDDetectionSystemClass ui;
};
