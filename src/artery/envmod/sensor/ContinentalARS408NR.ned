package artery.envmod.sensor;

simple ContinentalARS408NR extends RealisticRadarSensor
{
    fovRange @unit(m) = 100.0 m;
    fovAngle = 120.0; // degree
    rangeAccuracy @unit(m) = 0.1 m; 
    rangeResolution @unit(m) = 0.39 m;
    angleAccuracy = 1.0; // degree
    angleResolution = 4.5; // degree
    velocityAccuracy @unit(mps) = 0.02 mps;
    velocityResolution @unit(mps) = 0.11 mps; 
    minimalDistance @unit(m) = 0.2 m;
    attachmentPoint = "FRONT";
}