#pragma once
#include <iostream>
#include "ManagedObject.h"
#include "../ArtGalleryCore/WeakVisibility.h"
using namespace System;

public ref class WeakVisibilityCLR : public ManagedObject<WeakVisibility>
{
public: 
    bool initialize();
    bool compute();


    WeakVisibilityCLR(int tId);

    property String^ decompSVGString
    {
    public:
        String^ get()
        {
            return gcnew String(m_Instance->decompSVGString.c_str());
        }
    }
    property String^ initSVGString
    {
    public:
        String^ get()
        {
            return gcnew String(m_Instance->initSVGString.c_str());
        }
    }
    property String^ intermediateSVGString
    {
    public:
        String^ get()
        {
            return gcnew String(m_Instance->intermediateSVGString.c_str());
        }
    }
    property int testPolygonID
    {
    public:
        int get()
        {
            return m_Instance->testPolygonID;
        }
    private:
        void set(int value)
        {
        }
    }
    property int intermediateSteps
    {
    public:
        int get()
        {
            return m_Instance->intermediateSteps;
        }
    private:
        void set(int value)
        {
        }
    }
};


