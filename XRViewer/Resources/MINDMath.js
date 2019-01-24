function applyQuatToVect( q, vect ){
    var x = vect[0];
    var y = vect[1];
    var z = vect[2];
    
    var qx = q[0];
    var qy = q[1];
    var qz = q[2];
    var qw = q[3];
    
    var ix =  qw * x + qy * z - qz * y;
    var iy =  qw * y + qz * x - qx * z;
    var iz =  qw * z + qx * y - qy * x;
    var iw = - qx * x - qy * y - qz * z;
    
    return [ ix * qw + iw * - qx + iy * - qz - iz * - qy,
            iy * qw + iw * - qy + iz * - qx - ix * - qz,
            iz * qw + iw * - qz + ix * - qy - iy * - qx];
}


function quatFromEuler( euler ){
    return quatFromRotations(euler[0],euler[1],euler[2]);
}

function quatFromRotations( x, y, z ){
    var c1 = Math.cos( x * 0.5 );
    var c2 = Math.cos( y * 0.5 );
    var c3 = Math.cos( z * 0.5 );
    var s1 = Math.sin( x * 0.5 );
    var s2 = Math.sin( y * 0.5 );
    var s3 = Math.sin( z * 0.5 );
    
    return [s1 * c2 * c3 + c1 * s2 * s3,
            c1 * s2 * c3 - s1 * c2 * s3,
            c1 * c2 * s3 + s1 * s2 * c3,
            c1 * c2 * c3 - s1 * s2 * s3];
}

function quatFromUnitVectors( v1, v2 ) {
    var r = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2] + 1;
    if( r < 0.000001){
        r = 0;
        if ( Math.abs( v1[0] ) > Math.abs( v1[2] ) ){
            return makeNormalizedQuat( -v1[1], v1[0], 0, 1 );
        }
        else{
            return makeNormalizedQuat(0, - v1[2], v1[1], 1);
        }
    }
    else{
        return makeNormalizedQuat(v1[1] * v2[2] - v1[2] * v2[1],
                                  v1[2] * v2[0] - v1[0] * v2[2],
                                  v1[0] * v2[1] - v1[1] * v2[0],
                                  r);
    }
}

function makeNormalizedQuat(x,y,z,w){
    var l = Math.hypot(x,y,z,w);
    if( l === 0 ){
        return [0, 0, 0, 1];
    }
    else{
        l = 1 / l;
        return [x*l,y*l,z*l,w*l];
    }
}


function matrixToEuler( m ){
    var m11 = m[ 0 ], m12 = m[ 4 ], m13 = m[ 8 ];
    var m21 = m[ 1 ], m22 = m[ 5 ], m23 = m[ 9 ];
    var m31 = m[ 2 ], m32 = m[ 6 ], m33 = m[ 10 ];
    var y = Math.max(-1,Math.min(1,Math.asin(m13)));
    var x,z;
    if( Math.abs( m13 ) < 0.99999 ){
        x = Math.atan2( - m23, m33 );
        z = Math.atan2( - m12, m11 );
    }
    else{
        
        x = Math.atan2( m32, m22 );
        z = 0;
    }
    return [x,y,z];
}

function quatToEuler_TF( quat ){
    var mat = buildTransform([0,0,0], quat);
    return matrixToEuler(mat);
}

function quatToEuler(quat) {
    const q0 = quat[3];
    const q1 = quat[0];
    const q2 = quat[1];
    const q3 = quat[2];
    
    const Rx = Math.atan2(2 * (q0 * q1 + q2 * q3), 1 - (2 * (q1 * q1 + q2 * q2)));
    const Ry = Math.asin(2 * (q0 * q2 - q3 * q1));
    const Rz = Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - (2  * (q2 * q2 + q3 * q3)));
    return [Rx, Ry, Rz];
}


function buildTransform( pos, quat ) {
    var x = quat[0], y = quat[1], z = quat[2], w = quat[3];
    var x2 = x + x,    y2 = y + y, z2 = z + z;
    var xx = x * x2, xy = x * y2, xz = x * z2;
    var yy = y * y2, yz = y * z2, zz = z * z2;
    var wx = w * x2, wy = w * y2, wz = w * z2;
    
    return [1 - ( yy + zz ),
            xy + wz,
            xz - wy,
            0,
            xy - wz,
            1 - ( xx + zz ),
            yz + wx,
            0,
            xz + wy,
            yz - wx,
            1 - ( xx + yy ),
            0,
            pos[0],
            pos[1],
            pos[2],
            1];
}

function inverseQuat(quat){
    var x = -quat[0];
    var y = -quat[1];
    var z = -quat[2];
    var w = quat[3];
    return makeNormalizedQuat(x,y,z,w);
}

function reverseQuatRotations(quat){
    return multiplyQuatRotations(quat,[-1,-1,-1]);
}

function multiplyQuatRotations(quat, mult){
    var mat = buildTransform([0,0,0], quat);
    var euler = matrixToEuler(mat);
    euler[0]*=mult[0];
    euler[1]*=mult[1];
    euler[2]*=mult[2];
    return quatFromEuler(euler);
}

function multiplyQuaternions( a, b ) {
    var qax = a[0], qay = a[1], qaz = a[2], qaw = a[3];
    var qbx = b[0], qby = b[1], qbz = b[2], qbw = b[3];
    
    return [qax * qbw + qaw * qbx + qay * qbz - qaz * qby,
            qay * qbw + qaw * qby + qaz * qbx - qax * qbz,
            qaz * qbw + qaw * qbz + qax * qby - qay * qbx,
            qaw * qbw - qax * qbx - qay * qby - qaz * qbz];
}



function buildCoordinateSpaceTransform(newOrigin, newXAxis, newYAxis, newZAxis) {
    var tf = [     newXAxis[0], newYAxis[0], newZAxis[0], 0,
              newXAxis[1], newYAxis[1], newZAxis[1], 0,
              newXAxis[2], newYAxis[2], newZAxis[2], 0,
              0, 0, 0, 1 ];
    var t0 = transformPoint3D(tf,newOrigin);
    tf[12] = -t0[0];
    tf[13] = -t0[1];
    tf[14] = -t0[2];
    return tf;
}

function getQuaternionFromRightTriangle( originPoint, xPoint, yPoint ){
   var xAxis = subtractVectors(xPoint,originPoint);
    normalizeVector(xAxis,xAxis);
    var yAxis = subtractVectors(yPoint,originPoint);
    normalizeVector(yAxis,yAxis);
    var zAxis = crossVectors(xAxis,yAxis);
    
    var mat = [xAxis[0], yAxis[0], zAxis[0], 1,
               xAxis[1], yAxis[1], zAxis[1], 1,
               xAxis[2], yAxis[2], zAxis[2], 1,
               0, 0, 0, 1 ];
    return getQuaternionFromMatrix(mat);
}

function getLookAtMatrix( point, up ) {
    var dx = point[0];
    var dy = point[1];
    var dz = point[2];
    var length = Math.hypot(dx,dy,dz);
    if(length == 0){
        return [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
    }
    else{
        dx/=length;
        dy/=length;
        dz/=length;
    }
    //Up is assumed to be unit vector?
    var upx=up[0];
    var upy=up[1];
    var upz=up[2];
    var ax = upy * dz - upz * dy;
    var ay = upz * dx - upx * dz;
    var az = upx * dy - upy * dx;
    var alen = Math.hypot(ax,ay,az);
    
    if(alen == 0){
        dz += 0.0001;
        ax = upy * dz - upz * dy;
        ay = upz * dx - upx * dz;
        az = upx * dy - upy * dx;
        alen = Math.hypot(ax,ay,az);
    }
    ax/=alen;
    ay/=alen;
    az/=alen;
    
    var bx = dy * az - dz * ay;
    var by = dz * ax - dx * az;
    var bz = dx * ay - dy * ax;
    var blen = Math.hypot(bx,by,bz);
    bx/=blen;
    by/=blen;
    bz/=blen;
    
    return [ax,ay,az,0,  bx,by,bz,0,  dx,dy,dz,0, 0,0,0,1];
}

function getTranslationFromMatrix( mat ){
    return [mat[12],mat[13],mat[14]];
}

function getQuaternionFromMatrix( mat ){
    let quat = [0,0,0,1];
    let trace = mat[0] + mat[5] + mat[10];
    let S = 0;
    if (trace > 0) {
        S = Math.sqrt(trace + 1.0) * 2;
        quat[3] = 0.25 * S;
        quat[0] = (mat[6] - mat[9]) / S;
        quat[1] = (mat[8] - mat[2]) / S;
        quat[2] = (mat[1] - mat[4]) / S;
    } else if ((mat[0] > mat[5]) && (mat[0] > mat[10])) {
        S = Math.sqrt(1.0 + mat[0] - mat[5] - mat[10]) * 2;
        quat[3] = (mat[6] - mat[9]) / S;
        quat[0] = 0.25 * S;
        quat[1] = (mat[1] + mat[4]) / S;
        quat[2] = (mat[8] + mat[2]) / S;
    } else if (mat[5] > mat[10]) {
        S = Math.sqrt(1.0 + mat[5] - mat[0] - mat[10]) * 2;
        quat[3] = (mat[8] - mat[2]) / S;
        quat[0] = (mat[1] + mat[4]) / S;
        quat[1] = 0.25 * S;
        quat[2] = (mat[6] + mat[9]) / S;
    } else {
        S = Math.sqrt(1.0 + mat[10] - mat[0] - mat[5]) * 2;
        quat[3] = (mat[1] - mat[4]) / S;
        quat[0] = (mat[8] + mat[2]) / S;
        quat[1] = (mat[6] + mat[9]) / S;
        quat[2] = 0.25 * S;
    }
    return quat;
}

function transformPoint3D( matrix4, pt3d, outputPoint, defaultZValue ){
    defaultZValue = defaultZValue||0;
    var x = pt3d[0]||0, y = pt3d[1]||0;
    var z = pt3d[2];
    if(z == null){
        z = defaultZValue;
    }
    
    outputPoint = outputPoint || [];
    var m = matrix4;
    var s = 1/(m[ 3 ] * x + m[ 7 ] * y + m[ 11 ] * z + m[ 15 ]);
    outputPoint[0] = s*(m[ 0 ] * x + m[ 4 ] * y + m[ 8 ]  * z + m[ 12 ]);
    outputPoint[1] = s*(m[ 1 ] * x + m[ 5 ] * y + m[ 9 ]  * z + m[ 13 ]);
    outputPoint[2] = s*(m[ 2 ] * x + m[ 6 ] * y + m[ 10 ] * z + m[ 14 ]);
    return outputPoint;
}


function dotMultVectors( a, b ){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

function componentMultVectors( a, b, out ){
    out = out || [0,0,0];
    out[0] = a[0]*b[0];
    out[1] = a[1]*b[1];
    out[2] = a[2]*b[2];
    return out;
}

function getAngleBetweenVectors( a, b ){
    var alen = Math.hypot(a[0],a[1],a[2]);
    var blen = Math.hypot(b[0],b[1],b[2]);
    var dy = Math.hypot( a[0]*blen - b[0]*alen, a[1]*blen - b[1]*alen, a[2]*blen - b[2]*alen );
    var dx = Math.hypot( a[0]*blen + b[0]*alen, a[1]*blen + b[1]*alen, a[2]*blen + b[2]*alen );
    return Math.atan(dy/dx)*2;
}



function getQuaternionFromPositRotation(bestRotation){
    var euler = [-Math.asin(-bestRotation[1][2]),
                 -Math.atan2(bestRotation[0][2], bestRotation[2][2]),
                 Math.atan2(bestRotation[1][0], bestRotation[1][1])];
    return quatFromEuler(euler);
}

function getAngleBetweenQuaternions( a, b ){
    var innerProduct = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3] * b[3];
    var angle = Math.acos(2*innerProduct*innerProduct - 1);
    return angle;
}


function transformDeviceQuaternion_orig( tf, deviceQuaternion, deviceLocation ){
    var deviceMat = buildTransform( deviceLocation, deviceQuaternion );
    var newMat = multiplyMatrices4(tf,deviceMat);
    return getQuaternionFromMatrix(newMat);
}


function transformDeviceQuaternion( tf, deviceQuaternion, deviceLocation ){
    var deviceMat = buildTransform( deviceLocation, deviceQuaternion );
    var origin = transformPoint3D(tf,deviceLocation);
    var up = transformPoint3D(tf,transformPoint3D(deviceMat,[0,1,0]));
    var inFront = transformPoint3D(tf,transformPoint3D(deviceMat,[0,0,1]));
    up = subtractVectors(up,origin);
    inFront = subtractVectors(inFront,origin);
    
    var mat = getLookAtMatrix( inFront, up );
    return getQuaternionFromMatrix( mat );
}

function getClinkcodePose( clinkTagSize, deviceQuaternion, deviceLocation, tagPosData, cornersXY ){
    //var clinkTagSize = 10;//.0515;
    var error = tagPosData.bestError
    /*if(error > 20)
     {
     //TODO: we could see if we are viewing the tag straight on, which is where we get the highest errors
     //      in which case, we could match the deviceQuaternion with an extra z rot
     var minLength = Infinity;
     var maxLength = 0;
     for(var i=0; i<4; i++){
     var c0 = cornersXY[i];
     var c1 = cornersXY[(i+1)%4];
     var len = Math.hypot(c0.x-c1.x,c0.y-c1.y);
     minLength = Math.min(minLength,len);
     maxLength = Math.max(maxLength,len);
     }
     var minMaxRatio = minLength/maxLength
     console.log("best error: " + error + "     mmratio:" + minMaxRatio)
     }*/
    var screenQuat = getQuaternionFromPositRotation(tagPosData.bestRotation);
    var screenTrans = componentMultVectors(tagPosData.bestTranslation, [clinkTagSize, -clinkTagSize, -clinkTagSize]);
    
    var deltaScreenLoc = applyQuatToVect( deviceQuaternion , screenTrans );
    screenQuat = multiplyQuatRotations(screenQuat, [1,1,-1]);
    
    var screenTransform = buildTransform(screenTrans, screenQuat);
    var deviceTransform = buildTransform(deviceLocation, deviceQuaternion);
    
    var absTrans = multiplyMatrices4(deviceTransform, screenTransform);
    var absLoc = getTranslationFromMatrix(absTrans);
    var absQuat = getQuaternionFromMatrix(absTrans);
    
    return{
    quaternion:absQuat,
    position:absLoc,
    translation:deltaScreenLoc,
    error:[error]
    }
}

function makeClinkCoordinateTransform( deviceQuaternion, deviceLocation, screenPosData, clinkTagSize  ){
    var screenQuat = getQuaternionFromPositRotation(screenPosData.bestRotation);
    var screenTrans = componentMultVectors(screenPosData.bestTranslation, [clinkTagSize, -clinkTagSize, -clinkTagSize]);
    
    var deltaScreenLoc = applyQuatToVect( deviceQuaternion , screenTrans );
    var absoluteScreenLoc = addVectors(deviceLocation,deltaScreenLoc);
    
    screenQuat = multiplyQuatRotations(screenQuat, [1,1,-1]);
    
    var xPoint = addVectors(deviceLocation,
                            applyQuatToVect( deviceQuaternion ,
                                            addVectors(screenTrans,
                                                       applyQuatToVect( screenQuat ,[1,0,0]))));
    var yPoint = addVectors(deviceLocation,
                            applyQuatToVect( deviceQuaternion ,
                                            addVectors(screenTrans,
                                                       applyQuatToVect( screenQuat ,[0,-1,0]))));
    var zPoint = addVectors(deviceLocation,
                            applyQuatToVect( deviceQuaternion ,
                                            addVectors(screenTrans,
                                                       applyQuatToVect( screenQuat ,[0,0,-1]))));
    
    var xAxis = normalizeVector(subtractVectors(xPoint,absoluteScreenLoc));
    var yAxis = normalizeVector(subtractVectors(yPoint,absoluteScreenLoc));
    var zAxis = normalizeVector(subtractVectors(zPoint,absoluteScreenLoc));
    
    var rightsizeQuat = quatFromUnitVectors( yAxis, [0,1,0] );
    xAxis = applyQuatToVect(rightsizeQuat, xAxis);
    yAxis = applyQuatToVect(rightsizeQuat, yAxis);
    zAxis = applyQuatToVect(rightsizeQuat, zAxis);
    
    
    var coordinateTransform = buildCoordinateSpaceTransform(absoluteScreenLoc, xAxis, yAxis, zAxis);
    return {
    coordinateTransform: coordinateTransform,
    tagLocation:absoluteScreenLoc,
    tagQuaternion:screenQuat
    }
}

function getClinkBoardPose( roomTransform, deviceQuaternion, deviceLocation, boardPosData  ){
    //TODO: The transformed orientation is not correct yet.
    //      Probably shouldn't be using the transformDeviceQuaternion call and instead
    //      make a better routine for this case.
    
    var trans = boardPosData.bestTranslation;
    trans = [trans[0],-trans[1],-trans[2]];
    trans = applyQuatToVect( deviceQuaternion, trans );
    trans = addVectors(trans,deviceLocation);
    var orientation = getQuaternionFromPositRotation(boardPosData.bestRotation);
    orientation = multiplyQuaternions(deviceQuaternion,orientation);
    orientation = transformDeviceQuaternion(roomTransform, orientation, trans );
    trans = transformPoint3D( roomTransform, trans);
    
    //var mat = buildTransform(trans, orientation);
    return {
    position:trans,
    orientation:orientation
    }
}


function getRayFromCamera( cameraMatrix, projectionMatrix, screenLocXY, screenSizeXY  ){
    var rayOrigin = [cameraMatrix[12],cameraMatrix[13],cameraMatrix[14]];
    var iProjMat = invertMatrix4(projectionMatrix);
    multiplyMatrices4(cameraMatrix, iProjMat, iProjMat);
    
    var coordX = screenLocXY[0] / screenSizeXY[0] * 2 - 1;
    var coordY = screenLocXY[1] / screenSizeXY[1] * 2 - 1;
    
    var direction = [coordX, -coordY, 0.5];
    applyProjection( iProjMat, direction, direction );
    subtractVectors(direction, rayOrigin, direction);
    normalizeVector(direction,direction);
    return rayOrigin.concat(direction);
}

function rayPointAt( ray, t, out ) {
    out = out || [0,0,0];
    var d = [ray[3],ray[4],ray[5]];
    scaleVector(d,t,d);
    return addVectors(d,ray,d);
}

function addVectors( a, b, out) {
    out = out || [0,0,0];
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
    return out;
}

function subtractVectors(a, b, out) {
    out = out || [0,0,0];
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
    return out;
}

function vectorLengthSq(a){
    return a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
}

function vectorLength(a){
    return Math.hypot(a[0],a[1],a[2]);
}

function scaleVector(a, scale, out) {
    out = out || [0,0,0];
    out[0] = a[0] * scale;
    out[1] = a[1] * scale;
    out[2] = a[2] * scale;
    return out;
}

function distanceBetween(a, b) {
    let x = b[0] - a[0];
    let y = b[1] - a[1];
    let z = b[2] - a[2];
    return Math.hypot(x + y + z);
}


function normalizeVector(a, out) {
    out = out || [0,0,0];
    let x = a[0];
    let y = a[1];
    let z = a[2];
    let h = Math.hypot(x,y,z);
    if (h > 0) {
        h = 1 / h;
        out[0] = a[0] * h;
        out[1] = a[1] * h;
        out[2] = a[2] * h;
    }
    return out;
}


function crossVectors(a, b, out) {
    out = out || [0,0,0];
    let ax = a[0], ay = a[1], az = a[2];
    let bx = b[0], by = b[1], bz = b[2];
    out[0] = ay * bz - az * by;
    out[1] = az * bx - ax * bz;
    out[2] = ax * by - ay * bx;
    return out;
}


function applyProjection( m, v, out ) {
    var x = v[0], y = v[1], z = v[2];
    var d = 1 / ( m[ 3 ] * x + m[ 7 ] * y + m[ 11 ] * z + m[ 15 ] ); // perspective divide
    out = out || [0,0,0]
    out[0] = ( m[ 0 ] * x + m[ 4 ] * y + m[ 8 ]  * z + m[ 12 ] ) * d;
    out[1] = ( m[ 1 ] * x + m[ 5 ] * y + m[ 9 ]  * z + m[ 13 ] ) * d;
    out[2] = ( m[ 2 ] * x + m[ 6 ] * y + m[ 10 ] * z + m[ 14 ] ) * d;
    return this;
}

function invertMatrix4(a, out) {
    let a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
    let a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
    let a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
    let a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];
    let b00 = a00 * a11 - a01 * a10;
    let b01 = a00 * a12 - a02 * a10;
    let b02 = a00 * a13 - a03 * a10;
    let b03 = a01 * a12 - a02 * a11;
    let b04 = a01 * a13 - a03 * a11;
    let b05 = a02 * a13 - a03 * a12;
    let b06 = a20 * a31 - a21 * a30;
    let b07 = a20 * a32 - a22 * a30;
    let b08 = a20 * a33 - a23 * a30;
    let b09 = a21 * a32 - a22 * a31;
    let b10 = a21 * a33 - a23 * a31;
    let b11 = a22 * a33 - a23 * a32;
    let det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
    if (!det) {
        return null;
    }
    out = out || [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    det = 1.0 / det;
    out[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
    out[1] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
    out[2] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
    out[3] = (a22 * b04 - a21 * b05 - a23 * b03) * det;
    out[4] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
    out[5] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
    out[6] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
    out[7] = (a20 * b05 - a22 * b02 + a23 * b01) * det;
    out[8] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
    out[9] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
    out[10] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
    out[11] = (a21 * b02 - a20 * b04 - a23 * b00) * det;
    out[12] = (a11 * b07 - a10 * b09 - a12 * b06) * det;
    out[13] = (a00 * b09 - a01 * b07 + a02 * b06) * det;
    out[14] = (a31 * b01 - a30 * b03 - a32 * b00) * det;
    out[15] = (a20 * b03 - a21 * b01 + a22 * b00) * det;
    return out;
}

function multiplyMatrices4(a, b, out) {
    let a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
    let a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
    let a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
    let a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];
    let b0  = b[0], b1 = b[1], b2 = b[2], b3 = b[3];
    out = out || [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    
    out[0] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
    out[1] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
    out[2] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
    out[3] = b0*a03 + b1*a13 + b2*a23 + b3*a33;
    b0 = b[4]; b1 = b[5]; b2 = b[6]; b3 = b[7];
    out[4] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
    out[5] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
    out[6] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
    out[7] = b0*a03 + b1*a13 + b2*a23 + b3*a33;
    b0 = b[8]; b1 = b[9]; b2 = b[10]; b3 = b[11];
    out[8] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
    out[9] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
    out[10] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
    out[11] = b0*a03 + b1*a13 + b2*a23 + b3*a33;
    b0 = b[12]; b1 = b[13]; b2 = b[14]; b3 = b[15];
    out[12] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
    out[13] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
    out[14] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
    out[15] = b0*a03 + b1*a13 + b2*a23 + b3*a33;
    return out;
}



function generateProjectionMatrix(fovy, aspect, near, far, out) {
    let f = 1.0 / Math.tan(fovy / 2), nf;
    out = out || [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    out[0] = f / aspect;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[4] = 0;
    out[5] = f;
    out[6] = 0;
    out[7] = 0;
    out[8] = 0;
    out[9] = 0;
    out[11] = -1;
    out[12] = 0;
    out[13] = 0;
    out[15] = 0;
    if (far != null && far !== Infinity) {
        nf = 1 / (near - far);
        out[10] = (far + near) * nf;
        out[14] = (2 * far * near) * nf;
    } else {
        out[10] = -1;
        out[14] = -2 * near;
    }
    return out;
}


function rayDistanceToRay( rayA, rayB, maxDist, pointOnRayA, pointOnRayB ) {
    var segCenter = rayB;
    var segDir = rayB.slice(3);
    var diff = subtractVectors(rayA,segCenter);
    var segExtent = maxDist;
    var dirA = rayA.slice(3);
    var a01 = - dotMultVectors(dirA,segDir);
    var b0 = dotMultVectors(diff,dirA);
    var b1 = - dotMultVectors(diff,segDir);
    var c = vectorLengthSq(diff);
    var det = Math.abs( 1 - a01 * a01 );
    
    var s0, s1, sqrDist, extDet;
    
    if ( det > 0 ) {
        // The rays are not parallel.
        s0 = a01 * b1 - b0;
        s1 = a01 * b0 - b1;
        extDet = segExtent * det;
        
        if ( s0 >= 0 ) {
            if ( s1 >= - extDet ) {
                if ( s1 <= extDet ) {
                    // region 0
                    // Minimum at interior points of ray and segment.
                    var invDet = 1 / det;
                    s0 *= invDet;
                    s1 *= invDet;
                    sqrDist = s0 * ( s0 + a01 * s1 + 2 * b0 ) + s1 * ( a01 * s0 + s1 + 2 * b1 ) + c;
                }else {
                    // region 1
                    s1 = segExtent;
                    s0 = Math.max( 0, - ( a01 * s1 + b0 ) );
                    sqrDist = - s0 * s0 + s1 * ( s1 + 2 * b1 ) + c;
                }
                
            } else {
                // region 5
                s1 = - segExtent;
                s0 = Math.max( 0, - ( a01 * s1 + b0 ) );
                sqrDist = - s0 * s0 + s1 * ( s1 + 2 * b1 ) + c;
            }
        } else {
            if ( s1 <= - extDet ) {
                // region 4
                s0 = Math.max( 0, - ( - a01 * segExtent + b0 ) );
                s1 = ( s0 > 0 ) ? - segExtent : Math.min( Math.max( - segExtent, - b1 ), segExtent );
                sqrDist = - s0 * s0 + s1 * ( s1 + 2 * b1 ) + c;
                
            } else if ( s1 <= extDet ) {
                // region 3
                s0 = 0;
                s1 = Math.min( Math.max( - segExtent, - b1 ), segExtent );
                sqrDist = s1 * ( s1 + 2 * b1 ) + c;
                
            } else {
                // region 2
                s0 = Math.max( 0, - ( a01 * segExtent + b0 ) );
                s1 = ( s0 > 0 ) ? segExtent : Math.min( Math.max( - segExtent, - b1 ), segExtent );
                sqrDist = - s0 * s0 + s1 * ( s1 + 2 * b1 ) + c;
            }
        }
        
    } else {
        // Rays are parallel.
        s1 = ( a01 > 0 ) ? - segExtent : segExtent;
        s0 = Math.max( 0, - ( a01 * s1 + b0 ) );
        sqrDist = - s0 * s0 + s1 * ( s1 + 2 * b1 ) + c;
    }
    
    if ( pointOnRayA ) {
        scaleVector(dirA,s0,pointOnRayA);
        addVectors(pointOnRayA,rayA,pointOnRayA);
    }
    
    if ( pointOnRayB ) {
        scaleVector(segDir,s1,pointOnRayB);
        addVectors(pointOnRayB,segCenter,pointOnRayB);
    }
    
    //return Math.sqrt(sqrDist);
    return [s0,s1,Math.sqrt(sqrDist)];
}

function triangulatePointsFrom2CameraViews( pixW, pixH, cameraMatrixA, cameraProjectionA, screenPointsA, cameraMatrixB, cameraProjectionB, screenPointsB ){
    var widthHeight = [pixW,pixH];
    var distances = [];
    var pointsOnRayA = [];
    var pointsOnRayB = [];
    var nPoints = screenPointsA.length / 2;
    
    var ptA = [];
    var ptB = [];
    for(var i=0; i<nPoints; i++){
        var pointOffset = i*2;
        var rayA = getRayFromCamera( cameraMatrixA, cameraProjectionA, screenPointsA.slice(pointOffset,pointOffset+2), widthHeight );
        var rayB = getRayFromCamera( cameraMatrixB, cameraProjectionB, screenPointsB.slice(pointOffset,pointOffset+2), widthHeight );
        var dist = rayDistanceToRay( rayA, rayB, 20, ptA, ptB );
        
        /*if(i == 0){
            console.log("rayA: " + rayA)
            console.log("rayB: " + rayB)
            console.log("dist: " + dist)
            console.log("ang: " + getAngleBetweenVectors(rayA.slice(3), rayB.slice(3)))
        }*/
        distances.push.apply(distances,dist);
        pointsOnRayA.push.apply(pointsOnRayA,ptA);
        pointsOnRayB.push.apply(pointsOnRayB,ptB);
    }
    
    return {
        distances:distances,
        aPoints:pointsOnRayA,
        bPoints:pointsOnRayB
    }
}

"LOADED MINDMath.js"
