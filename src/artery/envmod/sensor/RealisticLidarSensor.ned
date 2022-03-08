//
// Artery V2X Simulation Framework
// Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.envmod.sensor;

simple RealisticLidarSensor like RealisticFovSensor
{
    parameters:
        @signal[RealisticFovSensorVisibleObjectPoints](type=long);
        @statistic[visibleObjectPoints](source=RealisticFovSensorVisibleObjectPoints; record=stats);
        @signal[RealisticFovSensorErasedPoints](type=long);
        @statistic[erasedObjectPoints](source=RealisticFovSensorErasedPoints; record=stats);

        @signal[RealisticFovSensorCombinedObjects](type=long);
        @statistic[combinedObjects](source=RealisticFovSensorCombinedObjects; record=stats, vector);
        @signal[RealisticFovSensorWrapperOutline](type=long);
        @statistic[wrapperOutline](source=RealisticFovSensorWrapperOutline; record=stats, vector);

        @signal[RealisticFovSensorOffCentreX](type=double);
        @statistic[offsetCentreX](source=RealisticFovSensorOffCentreX; record=stats);
        @signal[RealisticFovSensorOffCentreY](type=double);
        @statistic[offsetCentreY](source=RealisticFovSensorOffCentreY; record=stats);
        @signal[RealisticFovSensorOffCentreLength](type=double);
        @statistic[offsetCentreLength](source=RealisticFovSensorOffCentreLength; record=stats; unit=m);
        @signal[RealisticFovSensorDimension1](type=double);
        @statistic[dimension1](source=RealisticFovSensorDimension1; record=stats; unit=m);
        @signal[RealisticFovSensorDimension2](type=double);
        @statistic[dimension2](source=RealisticFovSensorDimension2; record=stats; unit=m);
        @signal[RealisticFovSensorObjectPointDistance](type=double);
        @statistic[objectPointSensorDistance](source=RealisticFovSensorObjectPointDistance; record=stats; unit=m);

        @signal[RealisticFovSensorRangErr](type=double);
        @statistic[rangeError](source=RealisticFovSensorRangErr; record=stats, vector; unit=m);
        
        double fovRange @unit(m) = default(80.0 m);
        double fovAngle = default(60.0); // degree
        string attachmentPoint = default("FRONT");
        int numSegments = default(1);
        bool doLineOfSightCheck = default(true);

        double angleAccuracy = 0.0; // degree //not used
        double rangeAccuracy @unit(m) = default(0.0 m); 
        double velocityAccuracy @unit(mps) = 0.0 mps; //not used
        double angleResolution = default(0.0); // degree
        double rangeResolution @unit(m) = 0.0 m;  //not used
        double velocityResolution @unit(mps) = 0.0 mps; //not used
        double minimalDistance @unit(m) = default(0.0 m);

        bool drawSensorCone = default(false);
        bool drawDetectedObjects = default(false);
        bool drawBlockingObstacles = default(false);
        bool drawLinesOfSight = default(false);
        bool drawObjectWrapper = default(false);
        bool drawResolution = default(false);
}
