#pragma once
#include <iostream>
#include "ManagedObject.h"
#include "../ArtGalleryCore/ArtGallery.h"


using namespace System;
using namespace System::Collections::Generic;

public ref class ArtGalleryCLR : public ManagedObject<ArtGallery>
{
public: 
    bool initialize();
    bool preProcess();
    bool iterations();
    void destroy();


    ArtGalleryCLR(int tId, int maxIterations, int criticalThreshold, bool useWeakVis, bool inTestMode, bool inDrawMode, int testSize);

    property String^ criticalSVGString
    {
    public:
        String^ get()
        {
            return gcnew String(m_Instance->criticalSVGString.c_str());
        }
    } property String^ decompSVGString
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
    
  
    property cli::array<double>^ hausdorffDistances
    {
    public:
        cli::array<double>^ get() 
        {
            cli::array<double>^ hDistances = gcnew cli::array<double>(m_Instance->hausdorffDistances.size());
            int i = 0;
            for (auto& dist : m_Instance->hausdorffDistances)
            {
                hDistances[i] = dist;
                i++;
            }

            return hDistances;
        }

    }

    property cli::array<double>^ p5Times
    {
    public:
        cli::array<double>^ get()
        {
            cli::array<double>^ p5Timez = gcnew cli::array<double>(m_Instance->p5Times.size());
            int i = 0;
            for (auto& time : m_Instance->p5Times)
            {
                p5Timez[i] = time;
                i++;
            }

            return p5Timez;
        }

    }


    property int intermediateSteps
    {
    public:
        int get()
        {
            return m_Instance->intermediateSteps;
        }
    
    }
   
    property int testPolygonID
    {
    public:
        int get()
        {
            return m_Instance->testPolygonID;
        }    
    }

    property int verticesAtEnd
    {
    public:
        int get()
        {
            return m_Instance->verticesAtEnd;
        }
    }

    property int facesAtEnd
    {
    public:
        int get()
        {
            return m_Instance->facesAtEnd;
        }
    }

    property int criticalVerticesAtEnd
    {
    public:
        int get()
        {
            return m_Instance->criticalVerticesAtEnd;
        }
    }

    property int criticalFacesAtEnd
    {
    public:
        int get()
        {
            return m_Instance->criticalFacesAtEnd;
        }
    }

    property int squareSplits
    {
    public:
        int get()
        {
            return m_Instance->squareSplits;
        }
    }

    property int angularSplits
    {
    public:
        int get()
        {
            return m_Instance->angularSplits;
        }
    }

    property int chordSplits
    {
    public:
        int get()
        {
            return m_Instance->chordSplits;
        }
    }

    property int extensionSplits
    {
    public:
        int get()
        {
            return m_Instance->extensionSplits;
        }
    }
    property int vislineSplits
    {
    public:
        int get()
        {
            return m_Instance->vislineSplits;
        }
    }
    property int solutionSize
    {
    public:
        int get()
        {
            return m_Instance->solutionSize;
        }
    }

    property double preprocessingTime
    {
    public:
        double get()
        {
            return m_Instance->preprocessingTime;
        }
    }

    property double processingTime
    {
    public:
        double get()
        {
            return m_Instance->processingTime;
        }
    }
    property int decompositionSize
    {
    public:
        int get()
        {
            return m_Instance->decompositionSize;
        }
    }

    property int maxDecompVertices
    {
    public:
        int get()
        {
            return m_Instance->maxDecompVertices;
        }
    }

    property int maxDecompReflexVertices
    {
    public:
        int get()
        {
            return m_Instance->maxDecompReflexVertices;
        }
    }

    property int weakvisLookups
    {
    public:
        int get()
        {
            return m_Instance->weakvisLookups;
        }
    }

    property int faceVisibilitiesComputed
    {
    public:
        int get()
        {
            return m_Instance->faceVisibilitiesComputed;
        }
    }

    property int pointVisibilitiesComputed
    {
    public:
        int get()
        {
            return m_Instance->pointVisibilitiesComputed;
        }
    }

    property int constantLookups
    {
    public:
        int get()
        {
            return m_Instance->constantLookups;
        }
    }
    property int currentGranularity
    {
    public:
        int get()
        {
            return m_Instance->currentGranularity;
        }
    }
    
    property double maxP5Time
    {
    public:
        void set(double maxP5Time)
        {
            m_Instance->maxP5Time = maxP5Time;
        }
    }


    property bool doingP5
    {
    public:
        void set(bool doingP5)
        {
            m_Instance->doingP5 = doingP5;
        }
    }

     



};


