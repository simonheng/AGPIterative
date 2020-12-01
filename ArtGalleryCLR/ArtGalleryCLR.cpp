#include "ArtGalleryCLR.h"

ArtGalleryCLR::ArtGalleryCLR(int tId, int maxIterations, int criticalThreshold, bool useWeakVis, bool inTestMode, bool inDrawMode, int testSize)
    : ManagedObject(new ArtGallery(tId, maxIterations, criticalThreshold, useWeakVis, inTestMode, inDrawMode, testSize))
{
    //Console::WriteLine("Creating a new Entity-wrapper object!");
}

void ArtGalleryCLR::destroy()
{
    m_Instance->~ArtGallery();
}


bool ArtGalleryCLR::iterations()
{
    return m_Instance->iterations();
}

bool ArtGalleryCLR::initialize()
{
    return m_Instance->initialize();
}

bool ArtGalleryCLR::preProcess()
{
    return m_Instance->preProcess();
}

