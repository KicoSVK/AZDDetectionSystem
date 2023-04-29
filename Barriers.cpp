#include "Barriers.h"

Barriers::Barriers() {
	Barriers::barriersState = Barriers::BarriersState::none;
}

void Barriers::setBarriersState(Barriers::BarriersState state)
{
	barriersState = state;
}

enum Barriers::BarriersState Barriers::getBarriersState()
{
	return Barriers::barriersState;
}

bool Barriers::isBarrierDown()
{
	switch (barriersState)
	{
	case Barriers::open:
		return 0;
		break;
	case Barriers::down:
		return 1;
		break;
	default:
		// Barriers::none;
		return -1;
		break;
	}
}
