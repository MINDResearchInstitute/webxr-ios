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

function normalizeVector(v){
    var l = Math.hypot(v[0],v[1],v[2]);
    if(l == 0){
        return [0,0,0];
    }
    else{
        l = 1/l;
        return [v[0]*l, v[1]*l, v[2]*l];
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

function quatToEuler( quat ){
    var mat = buildTransform([0,0,0], quat);
    return matrixToEuler(mat);
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

function multiplyMatrices4( a, b ) {
    return[
           a[0] * b[0] + a[4] * b[1] + a[8] * b[2] + a[12] * b[3],
           a[1] * b[0] + a[5] * b[1] + a[9] * b[2] + a[13] * b[3],
           a[2] * b[0] + a[6] * b[1] + a[10] * b[2] + a[14] * b[3],
           a[3] * b[0] + a[7] * b[1] + a[11] * b[2] + a[15] * b[3],
           a[0] * b[4] + a[4] * b[5] + a[8] * b[6] + a[12] * b[7],
           a[1] * b[4] + a[5] * b[5] + a[9] * b[6] + a[13] * b[7],
           a[2] * b[4] + a[6] * b[5] + a[10] * b[6] + a[14] * b[7],
           a[3] * b[4] + a[7] * b[5] + a[11] * b[6] + a[15] * b[7],
           a[0] * b[8] + a[4] * b[9] + a[8] * b[10] + a[12] * b[11],
           a[1] * b[8] + a[5] * b[9] + a[9] * b[10] + a[13] * b[11],
           a[2] * b[8] + a[6] * b[9] + a[10] * b[10] + a[14] * b[11],
           a[3] * b[8] + a[7] * b[9] + a[11] * b[10] + a[15] * b[11],
           a[0] * b[12] + a[4] * b[13] + a[8] * b[14] + a[12] * b[15],
           a[1] * b[12] + a[5] * b[13] + a[9] * b[14] + a[13] * b[15],
           a[2] * b[12] + a[6] * b[13] + a[10] * b[14] + a[14] * b[15],
           a[3] * b[12] + a[7] * b[13] + a[11] * b[14] + a[15] * b[15]];
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

function getQuatFromRotationMatrix( mat ){
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

function addVectors( a, b ){
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]];
}

function subtractVectors( a, b ){
    return [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
}

function dotMultVectors( a, b ){
    return [a[0]*b[0],a[1]*b[1],a[2]*b[2]];
}



function getQuaternionFromPositRotation(bestRotation){
    var euler = [-Math.asin(-bestRotation[1][2]),
                 -Math.atan2(bestRotation[0][2], bestRotation[2][2]),
                 Math.atan2(bestRotation[1][0], bestRotation[1][1])];
    return quatFromEuler(euler);
}


function transformDeviceQuaternion_orig( tf, deviceQuaternion, deviceLocation ){
    var deviceMat = buildTransform( deviceLocation, deviceQuaternion );
    var newMat = multiplyMatrices4(tf,deviceMat);
    return getQuatFromRotationMatrix(newMat);
}


function transformDeviceQuaternion( tf, deviceQuaternion, deviceLocation ){
    var deviceMat = buildTransform( deviceLocation, deviceQuaternion );
    var origin = transformPoint3D(tf,deviceLocation);
    var up = transformPoint3D(tf,transformPoint3D(deviceMat,[0,1,0]));
    var inFront = transformPoint3D(tf,transformPoint3D(deviceMat,[0,0,1]));
    up = subtractVectors(up,origin);
    inFront = subtractVectors(inFront,origin);
    
    var mat = getLookAtMatrix( inFront, up );
    return getQuatFromRotationMatrix( mat );
}


function makeClinkCoordinateTransform( deviceQuaternion, deviceLocation, screenPosData, clinkTagSize  ){
    var screenQuat = getQuaternionFromPositRotation(screenPosData.bestRotation);
    var screenTrans = dotMultVectors(screenPosData.bestTranslation, [clinkTagSize, -clinkTagSize, -clinkTagSize]);
    
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





/*
Posit1 and SVD
Copyright (c) 2012 Juan Mellado

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/*
References:
- "Numerical Recipes in C - Second Edition"
  http://www.nr.com/
*/

var SVD = SVD || {};

SVD.svdcmp = function(a, m, n, w, v){
  var flag, i, its, j, jj, k, l, nm,
      anorm = 0.0, c, f, g = 0.0, h, s, scale = 0.0, x, y, z, rv1 = [];
      
  //Householder reduction to bidiagonal form
  for (i = 0; i < n; ++ i){
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m){
      for (k = i; k < m; ++ k){
        scale += Math.abs( a[k][i] );
      }
      if (0.0 !== scale){
        for (k = i; k < m; ++ k){
          a[k][i] /= scale;
          s += a[k][i] * a[k][i];
        }
        f = a[i][i];
        g = -SVD.sign( Math.sqrt(s), f );
        h = f * g - s;
        a[i][i] = f - g;
        for (j = l; j < n; ++ j){
          for (s = 0.0, k = i; k < m; ++ k){
            s += a[k][i] * a[k][j];
          }
          f = s / h;
          for (k = i; k < m; ++ k){
            a[k][j] += f * a[k][i];
          }
        }
        for (k = i; k < m; ++ k){
          a[k][i] *= scale;
        }
      }
    }
    w[i] = scale * g;
    g = s = scale = 0.0;
    if ( (i < m) && (i !== n - 1) ){
      for (k = l; k < n; ++ k){
        scale += Math.abs( a[i][k] );
      }
      if (0.0 !== scale){
        for (k = l; k < n; ++ k){
          a[i][k] /= scale;
          s += a[i][k] * a[i][k];
        }
        f = a[i][l];
        g = -SVD.sign( Math.sqrt(s), f );
        h = f * g - s;
        a[i][l] = f - g;
        for (k = l; k < n; ++ k){
          rv1[k] = a[i][k] / h;
        }
        for (j = l; j < m; ++ j){
          for (s = 0.0, k = l; k < n; ++ k){
            s += a[j][k] * a[i][k];
          }
          for (k = l; k < n; ++ k){
            a[j][k] += s * rv1[k];
          }
        }
        for (k = l; k < n; ++ k){
          a[i][k] *= scale;
        }
      }
    }
    anorm = Math.max(anorm, ( Math.abs( w[i] ) + Math.abs( rv1[i] ) ) );
  }

  //Acumulation of right-hand transformation
  for (i = n - 1; i >= 0; -- i){
    if (i < n - 1){
      if (0.0 !== g){
        for (j = l; j < n; ++ j){
          v[j][i] = ( a[i][j] / a[i][l] ) / g;
        }
        for (j = l; j < n; ++ j){
          for (s = 0.0, k = l; k < n; ++ k){
            s += a[i][k] * v[k][j];
          }
          for (k = l; k < n; ++ k){
            v[k][j] += s * v[k][i];
          }
        }
      }
      for (j = l; j < n; ++ j){
        v[i][j] = v[j][i] = 0.0;
      }
    }
    v[i][i] = 1.0;
    g = rv1[i];
    l = i;
  }

  //Acumulation of left-hand transformation
  for (i = Math.min(n, m) - 1; i >= 0; -- i){
    l = i + 1;
    g = w[i];
    for (j = l; j < n; ++ j){
      a[i][j] = 0.0;
    }
    if (0.0 !== g){
      g = 1.0 / g;
      for (j = l; j < n; ++ j){
        for (s = 0.0, k = l; k < m; ++ k){
          s += a[k][i] * a[k][j];
        }
        f = (s / a[i][i]) * g;
        for (k = i; k < m; ++ k){
          a[k][j] += f * a[k][i];
        }
      }
      for (j = i; j < m; ++ j){
        a[j][i] *= g;
      }
    }else{
        for (j = i; j < m; ++ j){
          a[j][i] = 0.0;
        }
    }
    ++ a[i][i];
  }

  //Diagonalization of the bidiagonal form
  for (k = n - 1; k >= 0; -- k){
    for (its = 1; its <= 30; ++ its){
      flag = true;
      for (l = k; l >= 0; -- l){
        nm = l - 1;
        if ( Math.abs( rv1[l] ) + anorm === anorm ){
          flag = false;
          break;
        }
        if ( Math.abs( w[nm] ) + anorm === anorm ){
          break;
        }
      }
      if (flag){
        c = 0.0;
        s = 1.0;
        for (i = l; i <= k; ++ i){
          f = s * rv1[i];
          if ( Math.abs(f) + anorm === anorm ){
            break;
          }
          g = w[i];
          h = SVD.pythag(f, g);
          w[i] = h;
          h = 1.0 / h;
          c = g * h;
          s = -f * h;
          for (j = 1; j <= m; ++ j){
            y = a[j][nm];
            z = a[j][i];
            a[j][nm] = y * c + z * s;
            a[j][i] = z * c - y * s;
          }
        }
      }

      //Convergence
      z = w[k];
      if (l === k){
        if (z < 0.0){
          w[k] = -z;
          for (j = 0; j < n; ++ j){
            v[j][k] = -v[j][k];
          }
        }
        break;
      }

      if (30 === its){
        return false;
      }

      //Shift from bottom 2-by-2 minor
      x = w[l];
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ( (y - z) * (y + z) + (g - h) * (g + h) ) / (2.0 * h * y);
      g = SVD.pythag( f, 1.0 );
      f = ( (x - z) * (x + z) + h * ( (y / (f + SVD.sign(g, f) ) ) - h) ) / x;

      //Next QR transformation
      c = s = 1.0;
      for (j = l; j <= nm; ++ j){
        i = j + 1;
        g = rv1[i];
        y = w[i];
        h = s * g;
        g = c * g;
        z = SVD.pythag(f, h);
        rv1[j] = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y *= c;
        for (jj = 0; jj < n; ++ jj){
          x = v[jj][j];
          z = v[jj][i];
          v[jj][j] = x * c + z * s;
          v[jj][i] = z * c - x * s;
        }
        z = SVD.pythag(f, h);
        w[j] = z;
        if (0.0 !== z){
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        f = c * g + s * y;
        x = c * y - s * g;
        for (jj = 0; jj < m; ++ jj){
          y = a[jj][j];
          z = a[jj][i];
          a[jj][j] = y * c + z * s;
          a[jj][i] = z * c - y * s;
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }

  return true;
};

SVD.pythag = function(a, b){
  var at = Math.abs(a), bt = Math.abs(b), ct;

  if (at > bt){
    ct = bt / at;
    return at * Math.sqrt(1.0 + ct * ct);
  }
    
  if (0.0 === bt){
    return 0.0;
  }

  ct = at / bt;
  return bt * Math.sqrt(1.0 + ct * ct);
};

SVD.sign = function(a, b){
  return b >= 0.0? Math.abs(a): -Math.abs(a);
};


/*
References:
- "Iterative Pose Estimation using Coplanar Feature Points"
  Denis Oberkampf, Daniel F. DeMenthon, Larry S. Davis
  http://www.cfar.umd.edu/~daniel/daniel_papersfordownload/CoplanarPts.pdf
*/

var POS = POS || {};

POS.Posit = function(modelSize, focalLength){
  this.modelSize = modelSize;
  this.objectPoints = this.buildModel(modelSize);
  this.focalLength = focalLength;

  this.objectVectors = [];
  this.objectNormal = [];
  this.objectMatrix = [[],[],[]];
  
  this.init();
};

POS.Posit.prototype.buildModel = function(modelSize){
  var hWidth,hHeight;
  if(typeof modelSize == "number"){
  	hWidth = hHeight = modelSize / 2;
  }
  else{
  	//assume [width,height]
  	hWidth = modelSize[0]/2;
  	hHeight = modelSize[1]/2;
  }
  
  return [
    [-hWidth,  hHeight, 0.0],
    [ hWidth,  hHeight, 0.0],
    [ hWidth, -hHeight, 0.0],
    [-hWidth, -hHeight, 0.0] ];
};

POS.Posit.prototype.init = function(){
  var np = this.objectPoints.length,
      vectors = [], n = [], len = 0.0, row = 2, i;
  
  for (i = 0; i < np; ++ i){
    this.objectVectors[i] = [this.objectPoints[i][0] - this.objectPoints[0][0],
                             this.objectPoints[i][1] - this.objectPoints[0][1],
                             this.objectPoints[i][2] - this.objectPoints[0][2]];
                             
    vectors[i] = [this.objectVectors[i][0],
                  this.objectVectors[i][1],
                  this.objectVectors[i][2]];
  }

  while(0.0 === len){
    n[0] = this.objectVectors[1][1] * this.objectVectors[row][2] -
           this.objectVectors[1][2] * this.objectVectors[row][1];
    n[1] = this.objectVectors[1][2] * this.objectVectors[row][0] -
           this.objectVectors[1][0] * this.objectVectors[row][2];
    n[2] = this.objectVectors[1][0] * this.objectVectors[row][1] -
           this.objectVectors[1][1] * this.objectVectors[row][0];
    
    len = Math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    
    ++ row;
  }

  for (i = 0; i < 3; ++ i){
    this.objectNormal[i] = n[i] / len;
  }

  POS.pseudoInverse(vectors, np, this.objectMatrix);
};

POS.Posit.prototype.pose = function(imagePoints){
  var posRotation1 = [[],[],[]], posRotation2 = [[],[],[]], posTranslation = [],
      rotation1 = [[],[],[]], rotation2 = [[],[],[]], translation1 = [], translation2 = [],
      error1, error2, valid1, valid2, i, j;

  this.pos(imagePoints, posRotation1, posRotation2, posTranslation);

  valid1 = this.isValid(posRotation1, posTranslation);
  if (valid1){
    error1 = this.iterate(imagePoints, posRotation1, posTranslation, rotation1, translation1);
  }else{
    error1 = {euclidean: -1.0, pixels: -1, maximum: -1.0};
  }
  
  valid2 = this.isValid(posRotation2, posTranslation);
  if (valid2){
    error2 = this.iterate(imagePoints, posRotation2, posTranslation, rotation2, translation2);
  }else{
    error2 = {euclidean: -1.0, pixels: -1, maximum: -1.0};
  }

  for (i = 0; i < 3; ++ i){
    for (j = 0; j < 3; ++ j){
      if (valid1){
        translation1[i] -= rotation1[i][j] * this.objectPoints[0][j];
      }
      if (valid2){
        translation2[i] -= rotation2[i][j] * this.objectPoints[0][j];
      }
    }
  }

  return error1.euclidean < error2.euclidean?
    POS.Pose(error1.pixels, rotation1, translation1, error2.pixels, rotation2, translation2):
    POS.Pose(error2.pixels, rotation2, translation2, error1.pixels, rotation1, translation1);
};

POS.Posit.prototype.pos = function(imagePoints, rotation1, rotation2, translation){
  var np = this.objectPoints.length, imageVectors = [],
      i0 = [], j0 = [], ivec = [], jvec = [], row1 = [], row2 = [], row3 = [],
      i0i0, j0j0, i0j0, delta, q, lambda, mu, scale, i, j;

  for (i = 0; i < np; ++ i){
    imageVectors[i] = [imagePoints[i].x - imagePoints[0].x,
                       imagePoints[i].y - imagePoints[0].y];
  }

  //i0 and j0
  for (i = 0; i < 3; ++ i){
    i0[i] = 0.0;
    j0[i] = 0.0;
    for (j = 0; j < np; ++ j){
      i0[i] += this.objectMatrix[i][j] * imageVectors[j][0];
      j0[i] += this.objectMatrix[i][j] * imageVectors[j][1];
    }
  }

  i0i0 = i0[0] * i0[0] + i0[1] * i0[1] + i0[2] * i0[2];
  j0j0 = j0[0] * j0[0] + j0[1] * j0[1] + j0[2] * j0[2];
  i0j0 = i0[0] * j0[0] + i0[1] * j0[1] + i0[2] * j0[2];

  //Lambda and mu
  delta = (j0j0 - i0i0) * (j0j0 - i0i0) + 4.0 * (i0j0 * i0j0);
  
  if (j0j0 - i0i0 >= 0.0){
    q = (j0j0 - i0i0 + Math.sqrt(delta) ) / 2.0;
  }else{
    q = (j0j0 - i0i0 - Math.sqrt(delta) ) / 2.0;
  }
  
  if (q >= 0.0){
    lambda = Math.sqrt(q);
    if (0.0 === lambda){
      mu = 0.0;
    }else{
      mu = -i0j0 / lambda;
    }
  }else{
    lambda = Math.sqrt( -(i0j0 * i0j0) / q);
    if (0.0 === lambda){
      mu = Math.sqrt(i0i0 - j0j0);
    }else{
      mu = -i0j0 / lambda;
    }
  }

  //First rotation
  for (i = 0; i < 3; ++ i){
    ivec[i] = i0[i] + lambda * this.objectNormal[i];
    jvec[i] = j0[i] + mu * this.objectNormal[i];
  }
  
  scale = Math.sqrt(ivec[0] * ivec[0] + ivec[1] * ivec[1] + ivec[2] * ivec[2]);
  
  for (i = 0; i < 3; ++ i){
    row1[i] = ivec[i] / scale;
    row2[i] = jvec[i] / scale;
  }
  
  row3[0] = row1[1] * row2[2] - row1[2] * row2[1];
  row3[1] = row1[2] * row2[0] - row1[0] * row2[2];
  row3[2] = row1[0] * row2[1] - row1[1] * row2[0];

  for (i = 0; i < 3; ++ i){
    rotation1[0][i] = row1[i];
    rotation1[1][i] = row2[i];
    rotation1[2][i] = row3[i];
  }

  //Second rotation
  for (i = 0; i < 3; ++ i){
    ivec[i] = i0[i] - lambda * this.objectNormal[i];
    jvec[i] = j0[i] - mu * this.objectNormal[i];
  }
  
  for (i = 0; i < 3; ++ i){
    row1[i] = ivec[i] / scale;
    row2[i] = jvec[i] / scale;
  }
  
  row3[0] = row1[1] * row2[2] - row1[2] * row2[1];
  row3[1] = row1[2] * row2[0] - row1[0] * row2[2];
  row3[2] = row1[0] * row2[1] - row1[1] * row2[0];
  
  for (i = 0; i < 3; ++ i){
    rotation2[0][i] = row1[i];
    rotation2[1][i] = row2[i];
    rotation2[2][i] = row3[i];
  }

  //Translation
  translation[0] = imagePoints[0].x / scale;
  translation[1] = imagePoints[0].y / scale;
  translation[2] = this.focalLength / scale;
};

POS.Posit.prototype.isValid = function(rotation, translation){
  var np = this.objectPoints.length, zmin = Infinity, i = 0, zi;

  for (; i < np; ++ i){
    zi = translation[2] +
      (rotation[2][0] * this.objectVectors[i][0] +
       rotation[2][1] * this.objectVectors[i][1] +
       rotation[2][2] * this.objectVectors[i][2]);
    if (zi < zmin){
      zmin = zi;
    }
  }

  return zmin >= 0.0;
};

POS.Posit.prototype.iterate = function(imagePoints, posRotation, posTranslation, rotation, translation){
  var np = this.objectPoints.length,
      oldSopImagePoints = [], sopImagePoints = [],
      rotation1 = [[],[],[]], rotation2 = [[],[],[]],
      translation1 = [], translation2 = [],
      converged = false, iteration = 0,
      oldImageDifference, imageDifference, factor,
      error, error1, error2, delta, i, j;

  for (i = 0; i < np; ++ i){
    oldSopImagePoints[i] = {x: imagePoints[i].x,
                            y: imagePoints[i].y};
  }
  
  for (i = 0; i < 3; ++ i){
    for (j = 0; j < 3; ++ j){
      rotation[i][j] = posRotation[i][j];
    }
    translation[i] = posTranslation[i];
  }

  for (i = 0; i < np; ++ i){
    factor = 0.0;
    for (j = 0; j < 3; ++ j){
      factor += this.objectVectors[i][j] * rotation[2][j] / translation[2];
    }
    sopImagePoints[i] = {x: (1.0 + factor) * imagePoints[i].x,
                         y: (1.0 + factor) * imagePoints[i].y};
  }

  imageDifference = 0.0;
  
  for (i = 0; i < np; ++ i){
    imageDifference += Math.abs(sopImagePoints[i].x - oldSopImagePoints[i].x);
    imageDifference += Math.abs(sopImagePoints[i].y - oldSopImagePoints[i].y);
  }

  for (i = 0; i < 3; ++ i){
    translation1[i] = translation[i] -
      (rotation[i][0] * this.objectPoints[0][0] +
       rotation[i][1] * this.objectPoints[0][1] +
       rotation[i][2] * this.objectPoints[0][2]);
  }
  
  error = error1 = this.error(imagePoints, rotation, translation1);

  //Convergence
  converged = (0.0 === error1.pixels) || (imageDifference < 0.01);
  
  while( iteration ++ < 100 && !converged ){
  
    for (i = 0; i < np; ++ i){
      oldSopImagePoints[i].x = sopImagePoints[i].x;
      oldSopImagePoints[i].y = sopImagePoints[i].y;
    }

    this.pos(sopImagePoints, rotation1, rotation2, translation);

    for (i = 0; i < 3; ++ i){
      translation1[i] = translation[i] -
        (rotation1[i][0] * this.objectPoints[0][0] +
         rotation1[i][1] * this.objectPoints[0][1] +
         rotation1[i][2] * this.objectPoints[0][2]);
        
      translation2[i] = translation[i] -
        (rotation2[i][0] * this.objectPoints[0][0] +
         rotation2[i][1] * this.objectPoints[0][1] +
         rotation2[i][2] * this.objectPoints[0][2]);
    }

    error1 = this.error(imagePoints, rotation1, translation1);
    error2 = this.error(imagePoints, rotation2, translation2);

    if ( (error1.euclidean >= 0.0) && (error2.euclidean >= 0.0) ){
      if (error2.euclidean < error1.euclidean){
        error = error2;
        for (i = 0; i < 3; ++ i){
          for (j = 0; j < 3; ++ j){
            rotation[i][j] = rotation2[i][j];
          }
        }
      }else{
        error = error1;
        for (i = 0; i < 3; ++ i){
          for (j = 0; j < 3; ++ j){
            rotation[i][j] = rotation1[i][j];
          }
        }
      }
    }

    if ( (error1.euclidean < 0.0) && (error2.euclidean >= 0.0) ){
      error = error2;
      for (i = 0; i < 3; ++ i){
        for (j = 0; j < 3; ++ j){
          rotation[i][j] = rotation2[i][j];
        }
      }
    }
    
    if ( (error2.euclidean < 0.0) && (error1.euclidean >= 0.0) ){
      error = error1;
      for (i = 0; i < 3; ++ i){
        for (j = 0; j < 3; ++ j){
          rotation[i][j] = rotation1[i][j];
        }
      }
    }

    for (i = 0; i < np; ++ i){
      factor = 0.0;
      for (j = 0; j < 3; ++ j){
        factor += this.objectVectors[i][j] * rotation[2][j] / translation[2];
      }
      sopImagePoints[i].x = (1.0 + factor) * imagePoints[i].x;
      sopImagePoints[i].y = (1.0 + factor) * imagePoints[i].y;
    }

    oldImageDifference = imageDifference;
    imageDifference = 0.0;
    
    for (i = 0; i < np; ++ i){
      imageDifference += Math.abs(sopImagePoints[i].x - oldSopImagePoints[i].x);
      imageDifference += Math.abs(sopImagePoints[i].y - oldSopImagePoints[i].y);
    }

    delta = Math.abs(imageDifference - oldImageDifference);

    converged = (0.0 === error.pixels) || (delta < 0.01);
  }
  
  return error;
};

POS.Posit.prototype.error = function(imagePoints, rotation, translation){
  var np = this.objectPoints.length,
      move = [], projection = [], errorvec = [],
      euclidean = 0.0, pixels = 0.0, maximum = 0.0,
      i, j, k;

  if ( !this.isValid(rotation, translation) ){
    return {euclidean: -1.0, pixels: -1, maximum: -1.0};
  }
  
  for (i = 0; i < np; ++ i){
    move[i] = [];
    for (j = 0; j < 3; ++ j){
      move[i][j] = translation[j];
    }
  }
  
  for (i = 0; i < np; ++ i){
    for (j = 0; j < 3; ++ j){
      for (k = 0; k < 3; ++ k){
        move[i][j] += rotation[j][k] * this.objectPoints[i][k];
      }
    }
  }

  for (i = 0; i < np; ++ i){
    projection[i] = [];
    for (j = 0; j < 2; ++ j){
      projection[i][j] = this.focalLength * move[i][j] / move[i][2];
    }
  }
  
  for (i = 0; i < np; ++ i){
    errorvec[i] = [projection[i][0] - imagePoints[i].x,
                   projection[i][1] - imagePoints[i].y];
  }

  for (i = 0; i < np; ++ i){
    euclidean += Math.sqrt(errorvec[i][0] * errorvec[i][0] +
                           errorvec[i][1] * errorvec[i][1]);
                       
    pixels += Math.abs( Math.round(projection[i][0]) - Math.round(imagePoints[i].x) ) +
              Math.abs( Math.round(projection[i][1]) - Math.round(imagePoints[i].y) );
              
    if (Math.abs(errorvec[i][0]) > maximum){
      maximum = Math.abs(errorvec[i][0]);
    }
    if (Math.abs(errorvec[i][1]) > maximum){
      maximum = Math.abs(errorvec[i][1]);
    }
  }

  return {euclidean: euclidean / np, pixels: pixels, maximum: maximum};
};

POS.pseudoInverse = function(a, n, b){
  var w = [], v = [[],[],[]], s = [[],[],[]],
      wmax = 0.0, cn = 0,
      i, j, k;

  SVD.svdcmp(a, n, 3, w, v);

  for (i = 0; i < 3; ++ i){
    if (w[i] > wmax){
      wmax = w[i];
    }
  }

  wmax *= 0.01;

  for (i = 0; i < 3; ++ i){
    if (w[i] < wmax){
      w[i] = 0.0;
    }
  }

  for (j = 0; j < 3; ++ j){
    if (0.0 === w[j]){
      ++ cn;
      for (k = j; k < 2; ++ k){
        for (i = 0; i < n; ++ i){
          a[i][k] = a[i][k + 1];
        }
        for (i = 0; i < 3; ++ i){
          v[i][k] = v[i][k + 1];
        }
      }
    }
  }

  for (j = 0; j < 2; ++ j){
    if (0.0 === w[j]){
      w[j] = w[j + 1];
    }
  }

  for (i = 0; i < 3; ++ i){
    for (j = 0; j < 3 - cn; ++ j){
      s[i][j] = v[i][j] / w[j];
    }
  }
  
  for (i = 0; i < 3; ++ i){
    for (j = 0; j < n; ++ j){
      b[i][j] = 0.0;
      for (k = 0; k < 3 - cn; ++ k){
        b[i][j] += s[i][k] * a[j][k];
      }
    }
  }
};

POS.flattenArray = function( array ){
    var newArray = [];
    for( var i=0; i<array.length; i++ ){
        var subArray = array[i];
        if(Array.isArray(subArray)){
            newArray.push.apply(newArray,subArray);
        }
        else{
            newArray.push(subArray)
        }
    }
    return newArray;
}

POS.Pose = function(error1, rotation1, translation1, error2, rotation2, translation2){
    /*return {
        bestError: [error1],
        bestRotation: POS.flattenArray(rotation1),
        bestTranslation:translation1,
        alternativeError:[error2],
        alternativeRotation: POS.flattenArray(rotation2),
        alternativeTranslation:translation2
    }*/
    return {
        bestError: error1,
        bestRotation: rotation1,
        bestTranslation:translation1,
        alternativeError:error2,
        alternativeRotation: rotation2,
        alternativeTranslation:translation2
    };
};

var clinkcodePosit; //will get filled in

var getClinkcodeTransform = function( clinkTagSize, focalLength, corners, cameraMatrix ){
    var modelSize = 1;
    if(!clinkcodePosit || clinkcodePosit.focalLength != focalLength || clinkcodePosit.modelSize != modelSize){
        clinkcodePosit = new POS.Posit(modelSize, focalLength);
    }
    var pose = clinkcodePosit.pose([{x:corners[0], y:corners[1]},{x:corners[2], y:corners[3]},{x:corners[4], y:corners[5]},{x:corners[6], y:corners[7]}]);
    
    var cameraQuaternion = getQuatFromRotationMatrix(cameraMatrix);
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
        coordinateTransform:coordinateTransform
    };
}

"LOADED MINDXR.js"
