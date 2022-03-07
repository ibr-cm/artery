package artery.envmod.sensor;

simple BoschFrontRadar extends RealisticRadarSensor
{
    fovRange @unit(m) = 210.0 m;
    fovAngle = 120.0; // degree
    rangeAccuracy @unit(m) = 0.1 m; 
    rangeResolution @unit(m) = 0.2 m;
    angleAccuracy = 0.1; // degree
    angleResolution = 3; // degree
    velocityAccuracy @unit(mps) = 0.05 mps;
    velocityResolution @unit(mps) = 0.1 mps; 
    minimalDistance @unit(m) = 0.0 m;
    attachmentPoint = "FRONT";
}