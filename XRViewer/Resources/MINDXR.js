var clinkcodePosit; //will get filled in

function estimateClinkcodePose( focalLength, corners, cameraMatrix ){
    var modelSize = 1;
    if(!clinkcodePosit || clinkcodePosit.focalLength != focalLength || clinkcodePosit.modelSize != modelSize){
        clinkcodePosit = new POS.Posit(modelSize, focalLength);
    }
    var cornersXY = [{x:corners[0], y:corners[1]},{x:corners[2], y:corners[3]},{x:corners[4], y:corners[5]},{x:corners[6], y:corners[7]}];
    var pose = clinkcodePosit.pose(cornersXY);
    
    var cameraQuaternion = getQuaternionFromMatrix(cameraMatrix);
    var cameraLocation = [cameraMatrix[12],cameraMatrix[13],cameraMatrix[14]];
    
    var poseInfo = getClinkcodePose( 1, cameraQuaternion, cameraLocation, pose, cornersXY  );
    return poseInfo;
}

function getClinkcodeTransform( clinkTagSize, focalLength, corners, cameraMatrix ){
    var modelSize = 1;
    if(!clinkcodePosit || clinkcodePosit.focalLength != focalLength || clinkcodePosit.modelSize != modelSize){
        clinkcodePosit = new POS.Posit(modelSize, focalLength);
    }
    var cornersXY = [{x:corners[0], y:corners[1]},{x:corners[2], y:corners[3]},{x:corners[4], y:corners[5]},{x:corners[6], y:corners[7]}];
    var pose = clinkcodePosit.pose(cornersXY);
    
    var cameraQuaternion = getQuaternionFromMatrix(cameraMatrix);
    var cameraLocation = [cameraMatrix[12],cameraMatrix[13],cameraMatrix[14]];
    var transformInfo = makeClinkCoordinateTransform( cameraQuaternion, cameraLocation, pose, clinkTagSize  );
    var coordinateTransform = transformInfo.coordinateTransform;
    var tagLocation = transformInfo.tagLocation;
    var tagQuaternion = transformInfo.tagQuaternion;
    
    var roomPosition = transformPoint3D(coordinateTransform, cameraLocation);
    //var roomQuat = transformDeviceQuaternion(coordinateTransform, cameraQuaternion, cameraLocation);
    //var roomEuler = quatToEuler(roomQuat);
    
    return {
    transformedCameraPosition:roomPosition,
    tagLocation:tagLocation,
    tagQuaternion:tagQuaternion,
    tagOrientation:quatToEuler(tagQuaternion),
    coordinateTransform:coordinateTransform,
    tagOffset:pose.bestTranslation
    };
}

"LOADED MINDXR.js"
