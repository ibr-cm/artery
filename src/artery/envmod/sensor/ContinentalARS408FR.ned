package artery.envmod.sensor;

simple ContinentalARS408FR extends RealisticRadarSensor
{
    fovRange @unit(m) = 250.0 m;
    fovAngle = 18.0; // degree
    rangeAccuracy @unit(m) = 0.4 m; 
    rangeResolution @unit(m) = 1.79 m;
    angleAccuracy = 0.1; // degree
    angleResolution = 1.6; // degree
    velocityAccuracy @unit(mps) = 0.02 mps;
    velocityResolution @unit(mps) = 0.1 mps; 
    minimalDistance @unit(m) = 0.2 m;
    attachmentPoint = "FRONT";
}