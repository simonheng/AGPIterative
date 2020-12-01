#include "WeakVisibilityCLR.h"

WeakVisibilityCLR::WeakVisibilityCLR(int tId)
    : ManagedObject(new WeakVisibility(tId))     
{
    //Console::WriteLine("Creating a new Entity-wrapper object!");
}

bool WeakVisibilityCLR::initialize()
{
    return m_Instance->initialize();
}

bool WeakVisibilityCLR::compute()
{
    return m_Instance->compute();
}



