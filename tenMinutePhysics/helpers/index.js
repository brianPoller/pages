// import * as math from 'mathjs';
// import * as turf from "@turf/turf";
// import * as THREE from "three";
// import Delaunator from 'delaunator';
// import { useEffect, useState, useRef, createRef, forwardRef, Suspense } from 'react';
// import { Canvas, useFrame, useGraph, useThree } from '@react-three/fiber';
// import { Color, Plane, Vector3, MeshStandardMaterial, BufferGeometry, BufferAttribute, DoubleSide, Mesh } from 'three';
// import { Environment, Sphere, Cylinder, useGLTF } from '@react-three/drei';
// import { Physics, RigidBody, Debug, useSphericalJoint, useFixedJoint, useBeforePhysicsStep,
    //  useRevoluteJoint, useRapier, BallCollider, TrimeshCollider, interactionGroups, CylinderCollider, CuboidCollider, MeshCollider } from "@react-three/rapier";
// import { useRef } from 'react';
export const fovX = 1.3; // rad
export const fovY = 0.76; // rad or ? fovX*480/640;
let gridPointLength = null;
// const ballSpaceX=24; // resolution
// const ballSpaceY=24;
let numState = 32;
let numMeas = 36;
let RvalTheta = 0.01;
let RvalDepth = 0.01;
let ifovX = math.divide(-1, fovX);
let ifovY = math.divide(-1, fovY);
let Rnew = math.identity(numMeas);
Rnew = math.multiply(RvalTheta, Rnew);
Rnew.subset(math.index(2, 2), RvalDepth);
Rnew.subset(math.index(5, 5), RvalDepth);
Rnew.subset(math.index(8, 8), RvalDepth);
Rnew.subset(math.index(11, 11), RvalDepth);
Rnew.subset(math.index(14, 14), RvalDepth);
Rnew.subset(math.index(17, 17), RvalDepth);
Rnew.subset(math.index(20, 20), RvalDepth);
Rnew.subset(math.index(23, 23), RvalDepth);
Rnew.subset(math.index(26, 26), RvalDepth);
Rnew.subset(math.index(29, 29), RvalDepth);
Rnew.subset(math.index(32, 32), RvalDepth);
Rnew.subset(math.index(35, 35), RvalDepth);
let LSg = math.zeros(3, 1);
let RSg = math.zeros(3, 1);
let LHg = math.zeros(3, 1);
let RHg = math.zeros(3, 1);
let LEg = math.zeros(3, 1);
let REg = math.zeros(3, 1);
let LNg = math.zeros(3, 1);
let RNg = math.zeros(3, 1);
let LWg = math.zeros(3, 1);
let RWg = math.zeros(3, 1);
let LAg = math.zeros(3, 1);
let RAg = math.zeros(3, 1);
let rotMatLW = math.zeros(3, 3);
rotMatLW.subset(math.index(0, 0), 1);
let rotMatRW = math.zeros(3, 3);
rotMatRW.subset(math.index(0, 0), 1);
let rotMatLA = math.zeros(3, 3);
rotMatLA.subset(math.index(0, 0), 1);
let rotMatRA = math.zeros(3, 3);
rotMatRA.subset(math.index(0, 0), 1);
let dv1 = math.zeros(3, 3);
dv1.subset(math.index(1, 2), 1);
dv1.subset(math.index(2, 1), -1);
let dv2 = math.zeros(3, 3);
dv2.subset(math.index(0, 2), -1);
dv2.subset(math.index(2, 0), 1);
let dv3 = math.zeros(3, 3);
dv3.subset(math.index(0, 1), 1);
dv3.subset(math.index(1, 0), -1);
let dv4 = math.zeros(3, 1);
dv4.subset(math.index(1, 0), 1);
let dv5 = math.zeros(3, 1);
dv5.subset(math.index(0, 0), 0.5);
let dv6 = math.zeros(3, 1);
dv6.subset(math.index(1, 0), -1);
let dv7 = math.zeros(3, 1);
dv7.subset(math.index(0, 0), -0.5);
const projection = (vecIn, zin, cxin, cyin, alpha) => {
    let xv = math.subset(vecIn, math.index(0, 0));
    let yv = math.subset(vecIn, math.index(1, 0));
    let zv = math.subset(vecIn, math.index(2, 0));
    let xout = math.add(math.multiply(ifovX, math.divide(xv, math.subtract(zv, zin))), cxin);
    let yout = math.add(math.multiply(ifovY, math.divide(yv, math.subtract(zv, zin))), cyin);
    let zout = math.multiply(zv, alpha);
    return { x: xout, y: yout, z: zout };
};
export const quatFlip = (t1, t2, t3, t4, tempQuatNewin) => {
    let tt1 = math.multiply(t1, math.subset(tempQuatNewin, math.index(0, 0)));
    let tt2 = math.multiply(t2, math.subset(tempQuatNewin, math.index(0, 1)));
    let tt3 = math.multiply(t3, math.subset(tempQuatNewin, math.index(0, 2)));
    let tt4 = math.multiply(t4, math.subset(tempQuatNewin, math.index(0, 3)));
    let dquat0 = new math.matrix([tt1, tt2, tt3, tt4]);
    return { output: dquat0 };
};
export const quatCorrect = (t1, t2, t3, tempQuatNewin) => {
    let tt1 = math.divide(t1, 2);
    let tt2 = math.divide(t2, 2);
    let tt3 = math.divide(t3, 2);
    // let tt1 = math.divide(xhat.subset(math.index(0)),2); // tt1 end over end
    // let tt2 = math.divide(xhat.subset(math.index(1)),2);  // tt2 barrel roll
    // let tt3 = math.divide(xhat.subset(math.index(2)),2);
    let ggt = math.add(math.pow(tt1, 2), math.pow(tt2, 2), math.pow(tt3, 2));
    let tt4 = math.sqrt(math.subtract(1, ggt));
    let dquat0 = new math.matrix([tt1, tt2, tt3, tt4]);
    if (math.larger(ggt, 1)) {
        dquat0 = math.multiply(math.divide(1, math.add(1, math.sqrt(ggt))), math.matrix([tt1, tt2, tt3, 1]));
    }
    else {
        dquat0 = math.matrix([tt1, tt2, tt3, tt4]);
    }
    let tempQuat00 = quatMult(dquat0, tempQuatNewin).quat;
    return { output: tempQuat00 };
};
export const quatMultInv = (a, b) => {
    // inverse
    let qax = math.subset(a, math.index(0, 0));
    let qay = math.subset(a, math.index(0, 1));
    let qaz = math.subset(a, math.index(0, 2));
    let qaw = math.subset(a, math.index(0, 3));
    let qbx = math.multiply(math.subset(b, math.index(0, 0)), -1);
    let qby = math.multiply(math.subset(b, math.index(0, 1)), -1);
    let qbz = math.multiply(math.subset(b, math.index(0, 2)), -1);
    let qbw = math.subset(b, math.index(0, 3));
    let x = math.add(math.multiply(qax, qbw), math.multiply(qaw, qbx), math.multiply(qay, qbz), math.multiply(-1, math.multiply(qaz, qby)));
    let y = math.add(math.multiply(qay, qbw), math.multiply(qaw, qby), math.multiply(qaz, qbx), math.multiply(-1, math.multiply(qax, qbz)));
    let z = math.add(math.multiply(qaz, qbw), math.multiply(qaw, qbz), math.multiply(qax, qby), math.multiply(-1, math.multiply(qay, qbx)));
    let w = math.add(math.multiply(qaw, qbw), math.multiply(-1, math.multiply(qax, qbx)), math.multiply(-1, math.multiply(qay, qby)), math.multiply(-1, math.multiply(qaz, qbz)));
    let quatOut = math.matrix([[x, y, z, w]]);
    return { quat: quatOut };
};
export const quatMult2 = (a, b) => {
    let qax = math.subset(a, math.index(0, 0));
    let qay = math.subset(a, math.index(0, 1));
    let qaz = math.subset(a, math.index(0, 2));
    let qaw = math.subset(a, math.index(0, 3));
    let qbx = math.subset(b, math.index(0, 0));
    let qby = math.subset(b, math.index(0, 1));
    let qbz = math.subset(b, math.index(0, 2));
    let qbw = math.subset(b, math.index(0, 3));
    let x = math.add(math.multiply(qax, qbw), math.multiply(qaw, qbx), math.multiply(qay, qbz), math.multiply(-1, math.multiply(qaz, qby)));
    let y = math.add(math.multiply(qay, qbw), math.multiply(qaw, qby), math.multiply(qaz, qbx), math.multiply(-1, math.multiply(qax, qbz)));
    let z = math.add(math.multiply(qaz, qbw), math.multiply(qaw, qbz), math.multiply(qax, qby), math.multiply(-1, math.multiply(qay, qbx)));
    let w = math.add(math.multiply(qaw, qbw), math.multiply(-1, math.multiply(qax, qbx)), math.multiply(-1, math.multiply(qay, qby)), math.multiply(-1, math.multiply(qaz, qbz)));
    let quatOut = math.matrix([[x, y, z, w]]);
    return { quat: quatOut };
};
export const quatMult = (a, b) => {
    let qax = math.subset(a, math.index(0));
    let qay = math.subset(a, math.index(1));
    let qaz = math.subset(a, math.index(2));
    let qaw = math.subset(a, math.index(3));
    let qbx = math.subset(b, math.index(0, 0));
    let qby = math.subset(b, math.index(0, 1));
    let qbz = math.subset(b, math.index(0, 2));
    let qbw = math.subset(b, math.index(0, 3));
    let x = math.add(math.multiply(qax, qbw), math.multiply(qaw, qbx), math.multiply(qay, qbz), math.multiply(-1, math.multiply(qaz, qby)));
    let y = math.add(math.multiply(qay, qbw), math.multiply(qaw, qby), math.multiply(qaz, qbx), math.multiply(-1, math.multiply(qax, qbz)));
    let z = math.add(math.multiply(qaz, qbw), math.multiply(qaw, qbz), math.multiply(qax, qby), math.multiply(-1, math.multiply(qay, qbx)));
    let w = math.add(math.multiply(qaw, qbw), math.multiply(-1, math.multiply(qax, qbx)), math.multiply(-1, math.multiply(qay, qby)), math.multiply(-1, math.multiply(qaz, qbz)));
    let quatOut = math.matrix([[x, y, z, w]]);
    return { quat: quatOut };
};
export const LCGD2 = (quatIn) => {
    let theta1 = math.multiply(math.subset(quatIn, math.index(0, 0)), -1);
    let theta2 = math.multiply(math.subset(quatIn, math.index(0, 1)), -1);
    let theta3 = math.multiply(math.subset(quatIn, math.index(0, 2)), -1);
    let q4 = math.subset(quatIn, math.index(0, 3));
    let eye3 = math.identity(3);
    let wcross = new math.zeros(3, 3);
    wcross.subset(math.index(0, 1), -theta3);
    wcross.subset(math.index(0, 2), theta2);
    wcross.subset(math.index(1, 0), theta3);
    wcross.subset(math.index(1, 2), -theta1);
    wcross.subset(math.index(2, 0), -theta2);
    wcross.subset(math.index(2, 1), theta1);
    let wcross2 = math.multiply(wcross, wcross);
    wcross = math.multiply(2, wcross);
    wcross = math.multiply(q4, wcross);
    wcross2 = math.multiply(2, wcross2);
    let temp1 = math.subtract(eye3, wcross);
    let output = math.add(temp1, wcross2);
    return { LGC: output };
};
export const LCGD = (quatIn) => {
    let theta1 = math.subset(quatIn, math.index(0, 0));
    let theta2 = math.subset(quatIn, math.index(0, 1));
    let theta3 = math.subset(quatIn, math.index(0, 2));
    let q4 = math.subset(quatIn, math.index(0, 3));
    let eye3 = math.identity(3);
    let wcross = new math.zeros(3, 3);
    wcross.subset(math.index(0, 1), -theta3);
    wcross.subset(math.index(0, 2), theta2);
    wcross.subset(math.index(1, 0), theta3);
    wcross.subset(math.index(1, 2), -theta1);
    wcross.subset(math.index(2, 0), -theta2);
    wcross.subset(math.index(2, 1), theta1);
    let wcross2 = math.multiply(wcross, wcross);
    wcross = math.multiply(2, wcross);
    wcross = math.multiply(q4, wcross);
    wcross2 = math.multiply(2, wcross2);
    let temp1 = math.subtract(eye3, wcross);
    let output = math.add(temp1, wcross2);
    return { LGC: output };
};
const formH4 = (tempQuat, zin, SW, HW, L, alpha, LEquat, LE, REquat, RE, LNquat, LN, RNquat, RN, angLW, angRW, angLA, angRA) => {
    //  arguments are state parameters
    // SW HW L LE RE LN RN are state components stored here-- not returned to TryOn for rendering
    let H = new math.zeros(numState, numMeas);
    let rotMat = LCGD(tempQuat).LGC;
    let rotMatLE = LCGD(LEquat).LGC;
    let rotMatRE = LCGD(REquat).LGC;
    let rotMatLN = LCGD(LNquat).LGC;
    let rotMatRN = LCGD(RNquat).LGC;
    // I already assigned LSg, RSg etc. as basis vectors in forwardModel
    // and RotLW
    let LSr = math.multiply(rotMat, LSg);
    let RSr = math.multiply(rotMat, RSg);
    let LHr = math.multiply(rotMat, LHg);
    let RHr = math.multiply(rotMat, RHg);
    let LEl = math.add(LSg, math.multiply(rotMatLE, LEg));
    let REl = math.add(RSg, math.multiply(rotMatRE, REg));
    let LNl = math.add(LHg, math.multiply(rotMatLN, LNg));
    let RNl = math.add(RHg, math.multiply(rotMatRN, RNg));
    let LEr = math.multiply(rotMat, LEl);
    let REr = math.multiply(rotMat, REl);
    let LNr = math.multiply(rotMat, LNl);
    let RNr = math.multiply(rotMat, RNl);
    let LWlp = math.add(LEg, math.multiply(rotMatLW, LWg));
    let RWlp = math.add(REg, math.multiply(rotMatRW, RWg));
    let LAlp = math.add(LNg, math.multiply(rotMatLA, LAg));
    let RAlp = math.add(RNg, math.multiply(rotMatRA, RAg));
    let LWl = math.add(LSg, math.multiply(rotMatLE, LWlp));
    let RWl = math.add(RSg, math.multiply(rotMatRE, RWlp));
    let LAl = math.add(LHg, math.multiply(rotMatLN, LAlp));
    let RAl = math.add(RHg, math.multiply(rotMatRN, RAlp));
    let LWr = math.multiply(rotMat, LWlp);
    let RWr = math.multiply(rotMat, RWlp);
    let LAr = math.multiply(rotMat, LAlp);
    let RAr = math.multiply(rotMat, RAlp);
    let block1_1 = block1(rotMat, LSg, alpha, zin);
    let block1_2 = block1(rotMat, RSg, alpha, zin);
    let block1_3 = block1(rotMat, LHg, alpha, zin);
    let block1_4 = block1(rotMat, RHg, alpha, zin);
    let block1_5 = block1(rotMat, LEl, alpha, zin);
    let block1_6 = block1(rotMat, REl, alpha, zin);
    let block1_7 = block1(rotMat, LNl, alpha, zin);
    let block1_8 = block1(rotMat, RNl, alpha, zin);
    let block1_9 = block1(rotMat, LWl, alpha, zin);
    let block1_10 = block1(rotMat, RWl, alpha, zin);
    let block1_11 = block1(rotMat, LAl, alpha, zin);
    let block1_12 = block1(rotMat, RAl, alpha, zin);
    H.subset(math.index([0, 1, 2, 3], [0, 1, 2]), block1_1.Hpart);
    H.subset(math.index([0, 1, 2, 3], [3, 4, 5]), block1_2.Hpart);
    H.subset(math.index([0, 1, 2, 3], [6, 7, 8]), block1_3.Hpart);
    H.subset(math.index([0, 1, 2, 3], [9, 10, 11]), block1_4.Hpart);
    H.subset(math.index([0, 1, 2, 3], [12, 13, 14]), block1_5.Hpart);
    H.subset(math.index([0, 1, 2, 3], [15, 16, 17]), block1_6.Hpart);
    H.subset(math.index([0, 1, 2, 3], [18, 19, 20]), block1_7.Hpart);
    H.subset(math.index([0, 1, 2, 3], [21, 22, 23]), block1_8.Hpart);
    H.subset(math.index([0, 1, 2, 3], [24, 25, 26]), block1_9.Hpart);
    H.subset(math.index([0, 1, 2, 3], [27, 28, 29]), block1_10.Hpart);
    H.subset(math.index([0, 1, 2, 3], [30, 31, 32]), block1_11.Hpart);
    H.subset(math.index([0, 1, 2, 3], [33, 34, 35]), block1_12.Hpart);
    H.subset(math.index(7, 2), block1_1.alphaPart);
    H.subset(math.index(7, 5), block1_2.alphaPart);
    H.subset(math.index(7, 8), block1_3.alphaPart);
    H.subset(math.index(7, 11), block1_4.alphaPart);
    H.subset(math.index(7, 14), block1_5.alphaPart);
    H.subset(math.index(7, 17), block1_6.alphaPart);
    H.subset(math.index(7, 20), block1_7.alphaPart);
    H.subset(math.index(7, 23), block1_8.alphaPart);
    H.subset(math.index(7, 26), block1_9.alphaPart);
    H.subset(math.index(7, 29), block1_10.alphaPart);
    H.subset(math.index(7, 32), block1_11.alphaPart);
    H.subset(math.index(7, 35), block1_12.alphaPart);
    let block2_8 = block2(rotMat, rotMatLE, LSg, LEg, alpha, zin);
    let block2_9 = block2(rotMat, rotMatRE, RSg, REg, alpha, zin);
    let block2_10 = block2(rotMat, rotMatLN, LHg, LNg, alpha, zin);
    let block2_11 = block2(rotMat, rotMatRN, RHg, RNg, alpha, zin);
    let block2_12 = block2(rotMat, rotMatLE, LSg, LWlp, alpha, zin);
    let block2_13 = block2(rotMat, rotMatRE, RSg, RWlp, alpha, zin);
    let block2_14 = block2(rotMat, rotMatLN, LHg, LAlp, alpha, zin);
    let block2_15 = block2(rotMat, rotMatRN, RHg, RAlp, alpha, zin);
    H.subset(math.index([8, 9, 10, 11], [12, 13, 14]), block2_8.Hpart);
    H.subset(math.index([12, 13, 14, 15], [15, 16, 17]), block2_9.Hpart);
    H.subset(math.index([16, 17, 18, 19], [18, 19, 20]), block2_10.Hpart);
    H.subset(math.index([20, 21, 22, 23], [21, 22, 23]), block2_11.Hpart);
    H.subset(math.index([8, 9, 10, 11], [24, 25, 26]), block2_12.Hpart);
    H.subset(math.index([12, 13, 14, 15], [27, 28, 29]), block2_13.Hpart);
    H.subset(math.index([16, 17, 18, 19], [30, 31, 32]), block2_14.Hpart);
    H.subset(math.index([20, 21, 22, 23], [33, 34, 35]), block2_15.Hpart);
    let block3_1 = block3(rotMat, LSr, RSr, LHr, RHr, alpha, zin);
    let block3_2 = block3(rotMat, LEr, REr, LNr, RNr, alpha, zin);
    let block3_3 = block3(rotMat, LWr, RWr, LAr, RAr, alpha, zin);
    H.subset(math.index([4, 5, 6], [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]), block3_1.Hpart);
    H.subset(math.index([4, 5, 6], [12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]), block3_2.Hpart);
    H.subset(math.index([4, 5, 6], [24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35]), block3_3.Hpart);
    let block4_1 = block4(rotMat, rotMatLE, LSg, LEg, LWg, angLW, alpha, zin);
    let block4_2 = block4(rotMat, rotMatRE, RSg, REg, RWg, angRW, alpha, zin);
    let block4_3 = block4(rotMat, rotMatLN, LHg, LNg, LAg, angLA, alpha, zin);
    let block4_4 = block4(rotMat, rotMatRN, RHg, RNg, RAg, angRA, alpha, zin);
    H.subset(math.index([24, 25], [24, 25, 26]), block4_1.Hpart);
    H.subset(math.index([26, 27], [27, 28, 29]), block4_2.Hpart);
    H.subset(math.index([28, 29], [30, 31, 32]), block4_3.Hpart);
    H.subset(math.index([30, 31], [33, 34, 35]), block4_4.Hpart);
    return { output: H };
};
const block1 = (rotIn, vecIn, alpha, zin) => {
    let vecdt1 = math.multiply(rotIn, dv1, vecIn);
    let dxdt1 = math.subset(vecdt1, math.index(0, 0));
    let dydt1 = math.subset(vecdt1, math.index(1, 0));
    let dzdt1 = math.subset(vecdt1, math.index(2, 0));
    let vecdt2 = math.multiply(rotIn, dv2, vecIn);
    let dxdt2 = math.subset(vecdt2, math.index(0, 0));
    let dydt2 = math.subset(vecdt2, math.index(1, 0));
    let dzdt2 = math.subset(vecdt2, math.index(2, 0));
    let vecdt3 = math.multiply(rotIn, dv3, vecIn);
    let dxdt3 = math.subset(vecdt3, math.index(0, 0));
    let dydt3 = math.subset(vecdt3, math.index(1, 0));
    let dzdt3 = math.subset(vecdt3, math.index(2, 0));
    let vecOut = math.multiply(rotIn, vecIn);
    let xl = math.subset(vecOut, math.index(0, 0));
    let yl = math.subset(vecOut, math.index(1, 0));
    let zl = math.subset(vecOut, math.index(2, 0));
    let zld = math.subtract(zl, zin);
    let zld2 = math.pow(zld, 2);
    let H1_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(zld, dxdt1), math.multiply(xl, dzdt1)), zld2));
    let H1_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(zld, dydt1), math.multiply(yl, dzdt1)), zld2));
    let H1_3 = math.multiply(dzdt1, alpha);
    let H2_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(zld, dxdt2), math.multiply(xl, dzdt2)), zld2));
    let H2_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(zld, dydt2), math.multiply(yl, dzdt2)), zld2));
    let H2_3 = math.multiply(dzdt2, alpha);
    let H3_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(zld, dxdt3), math.multiply(xl, dzdt3)), zld2));
    let H3_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(zld, dydt3), math.multiply(yl, dzdt3)), zld2));
    let H3_3 = math.multiply(dzdt3, alpha);
    let H4_1 = math.multiply(ifovX, math.divide(xl, zld2));
    let H4_2 = math.multiply(ifovY, math.divide(yl, zld2));
    let H4_3 = 0;
    let alphaPart = zl;
    let H = math.matrix([
        [H1_1, H1_2, H1_3],
        [H2_1, H2_2, H2_3],
        [H3_1, H3_2, H3_3],
        [H4_1, H4_2, H4_3],
    ]);
    return { Hpart: H, alphaPart: alphaPart };
};
const block2 = (rot1, rot2, vec1, vec2, alpha, zin) => {
    let vecdt1 = math.multiply(rot1, rot2, dv1, vec2);
    let dxdt1 = math.subset(vecdt1, math.index(0, 0));
    let dydt1 = math.subset(vecdt1, math.index(1, 0));
    let dzdt1 = math.subset(vecdt1, math.index(2, 0));
    let vecdt2 = math.multiply(rot1, rot2, dv2, vec2);
    let dxdt2 = math.subset(vecdt2, math.index(0, 0));
    let dydt2 = math.subset(vecdt2, math.index(1, 0));
    let dzdt2 = math.subset(vecdt2, math.index(2, 0));
    let vecdt3 = math.multiply(rot1, rot2, dv3, vec2);
    let dxdt3 = math.subset(vecdt3, math.index(0, 0));
    let dydt3 = math.subset(vecdt3, math.index(1, 0));
    let dzdt3 = math.subset(vecdt3, math.index(2, 0));
    let vecdt4 = math.multiply(rot1, rot2, dv4);
    let dxdt4 = math.subset(vecdt4, math.index(0, 0));
    let dydt4 = math.subset(vecdt4, math.index(1, 0));
    let dzdt4 = math.subset(vecdt4, math.index(2, 0));
    let vecOut = math.multiply(rot1, math.add(vec1, math.multiply(rot2, vec2)));
    let xl = math.subset(vecOut, math.index(0, 0));
    let yl = math.subset(vecOut, math.index(1, 0));
    let zl = math.subset(vecOut, math.index(2, 0));
    let zld = math.subtract(zl, zin);
    let zld2 = math.pow(zld, 2);
    let H1_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(zld, dxdt1), math.multiply(xl, dzdt1)), zld2));
    let H1_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(zld, dydt1), math.multiply(yl, dzdt1)), zld2));
    let H1_3 = math.multiply(dzdt1, alpha);
    let H2_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(zld, dxdt2), math.multiply(xl, dzdt2)), zld2));
    let H2_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(zld, dydt2), math.multiply(yl, dzdt2)), zld2));
    let H2_3 = math.multiply(dzdt2, alpha);
    let H3_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(zld, dxdt3), math.multiply(xl, dzdt3)), zld2));
    let H3_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(zld, dydt3), math.multiply(yl, dzdt3)), zld2));
    let H3_3 = math.multiply(dzdt3, alpha);
    let H4_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(zld, dxdt4), math.multiply(xl, dzdt4)), zld2));
    let H4_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(zld, dydt4), math.multiply(yl, dzdt4)), zld2));
    let H4_3 = math.multiply(dzdt4, alpha);
    let H = math.matrix([
        [H1_1, H1_2, H1_3],
        [H2_1, H2_2, H2_3],
        [H3_1, H3_2, H3_3],
        [H4_1, H4_2, H4_3],
    ]);
    return { Hpart: H };
};
const block3 = (rot1, vec1, vec2, vec3, vec4, alpha, zin) => {
    let vecdt1 = math.multiply(rot1, dv5);
    let dxdt1 = math.subset(vecdt1, math.index(0, 0));
    let dydt1 = math.subset(vecdt1, math.index(1, 0));
    let dzdt1 = math.subset(vecdt1, math.index(2, 0));
    let vecdt2 = math.multiply(rot1, dv6);
    let dxdt2 = math.subset(vecdt2, math.index(0, 0));
    let dydt2 = math.subset(vecdt2, math.index(1, 0));
    let dzdt2 = math.subset(vecdt2, math.index(2, 0));
    let vecdt3 = math.multiply(rot1, dv7);
    let dxdt3 = math.subset(vecdt3, math.index(0, 0));
    let dydt3 = math.subset(vecdt3, math.index(1, 0));
    let dzdt3 = math.subset(vecdt3, math.index(2, 0));
    let vecdt4 = math.multiply(rot1, dv4);
    let dxdt4 = math.subset(vecdt4, math.index(0, 0));
    let dydt4 = math.subset(vecdt4, math.index(1, 0));
    let dzdt4 = math.subset(vecdt4, math.index(2, 0));
    //let vecOut=math.multiply(rot1,math.add(vec1, math.multiply(rot2,vec2)));
    let x1 = math.subset(vec1, math.index(0, 0));
    let y1 = math.subset(vec1, math.index(1, 0));
    let z1 = math.subset(vec1, math.index(2, 0));
    let z1d = math.subtract(z1, zin);
    let z1d2 = math.pow(z1d, 2);
    // vec1 --dv5, vecdt1
    let H1_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(z1d, dxdt1), math.multiply(x1, dzdt1)), z1d2));
    let H1_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(z1d, dydt1), math.multiply(y1, dzdt1)), z1d2));
    let H1_3 = math.multiply(dzdt1, alpha);
    // vec1 --dv6 vecdt2
    let H3_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(z1d, dxdt2), math.multiply(x1, dzdt2)), z1d2));
    let H3_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(z1d, dydt2), math.multiply(y1, dzdt2)), z1d2));
    let H3_3 = math.multiply(dzdt2, alpha);
    let x2 = math.subset(vec2, math.index(0, 0));
    let y2 = math.subset(vec2, math.index(1, 0));
    let z2 = math.subset(vec2, math.index(2, 0));
    let z2d = math.subtract(z2, zin);
    let z2d2 = math.pow(z2d, 2);
    // vec2 --dv7 vecdt3
    let H1_4 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(z2d, dxdt3), math.multiply(x2, dzdt3)), z2d2));
    let H1_5 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(z2d, dydt3), math.multiply(y2, dzdt3)), z2d2));
    let H1_6 = math.multiply(dzdt3, alpha);
    // vec2  --dv6 vect2d
    let H3_4 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(z2d, dxdt2), math.multiply(x2, dzdt2)), z2d2));
    let H3_5 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(z2d, dydt2), math.multiply(y2, dzdt2)), z2d2));
    let H3_6 = math.multiply(dzdt2, alpha);
    let x3 = math.subset(vec3, math.index(0, 0));
    let y3 = math.subset(vec3, math.index(1, 0));
    let z3 = math.subset(vec3, math.index(2, 0));
    let z3d = math.subtract(z3, zin);
    let z3d2 = math.pow(z3d, 2);
    // vec3  --dv5 vecdt1
    let H2_7 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(z3d, dxdt1), math.multiply(x3, dzdt1)), z3d2));
    let H2_8 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(z3d, dydt1), math.multiply(y3, dzdt1)), z3d2));
    let H2_9 = math.multiply(dzdt1, alpha);
    let x4 = math.subset(vec4, math.index(0, 0));
    let y4 = math.subset(vec4, math.index(1, 0));
    let z4 = math.subset(vec4, math.index(2, 0));
    let z4d = math.subtract(z4, zin);
    let z4d2 = math.pow(z4d, 2);
    // vec4 -- dv7 vecdt3
    let H2_10 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(z4d, dxdt3), math.multiply(x4, dzdt3)), z4d2));
    let H2_11 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(z4d, dydt3), math.multiply(y4, dzdt3)), z4d2));
    let H2_12 = math.multiply(dzdt3, alpha);
    let H = math.matrix([
        [H1_1, H1_2, H1_3, H1_4, H1_5, H1_6, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, H2_7, H2_8, H2_9, H2_10, H2_11, H2_12],
        [H3_1, H3_2, H3_3, H3_4, H3_5, H3_6, 0, 0, 0, 0, 0, 0]
    ]);
    return { Hpart: H };
};
const block4 = (rot1, rot2, vec1, vec2, vec3, ang, alpha, zin) => {
    let rot3 = math.zeros(3, 3);
    rot3.subset(math.index(0, 0), 1);
    rot3.subset(math.index(1, 1), math.cos(ang));
    rot3.subset(math.index(1, 2), math.multiply(-1, math.sin(ang)));
    rot3.subset(math.index(2, 1), math.sin(ang));
    rot3.subset(math.index(2, 2), math.cos(ang));
    let dv1 = math.zeros(3, 3);
    dv1.subset(math.index(1, 1), math.multiply(-1, math.sin(ang)));
    dv1.subset(math.index(1, 2), math.multiply(-1, math.cos(ang)));
    dv1.subset(math.index(2, 1), math.cos(ang));
    dv1.subset(math.index(2, 2), math.multiply(-1, math.sin(ang)));
    let vecdt1 = math.multiply(rot1, rot2, dv1, vec3);
    let dxdt1 = math.subset(vecdt1, math.index(0, 0));
    let dydt1 = math.subset(vecdt1, math.index(1, 0));
    let dzdt1 = math.subset(vecdt1, math.index(2, 0));
    let lvec = math.zeros(3, 1);
    lvec.subset(math.index(1, 0), 1);
    let vecdt4 = math.multiply(rot1, rot2, rot3, lvec);
    let dxdt4 = math.subset(vecdt4, math.index(0, 0));
    let dydt4 = math.subset(vecdt4, math.index(1, 0));
    let dzdt4 = math.subset(vecdt4, math.index(2, 0));
    let vecOut = math.multiply(rot1, math.add(vec1, math.multiply(rot2, math.add(vec2, math.multiply(rot3, vec3)))));
    let xl = math.subset(vecOut, math.index(0, 0));
    let yl = math.subset(vecOut, math.index(1, 0));
    let zl = math.subset(vecOut, math.index(2, 0));
    let zld = math.subtract(zl, zin);
    let zld2 = math.pow(zld, 2);
    let H1_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(zld, dxdt1), math.multiply(xl, dzdt1)), zld2));
    let H1_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(zld, dydt1), math.multiply(yl, dzdt1)), zld2));
    let H1_3 = math.multiply(dzdt1, alpha);
    let H2_1 = math.multiply(ifovX, math.divide(math.subtract(math.multiply(zld, dxdt4), math.multiply(xl, dzdt4)), zld2));
    let H2_2 = math.multiply(ifovY, math.divide(math.subtract(math.multiply(zld, dydt4), math.multiply(yl, dzdt4)), zld2));
    let H2_3 = math.multiply(dzdt4, alpha);
    let H = math.matrix([
        [H1_1, H1_2, H1_3],
        [H2_1, H2_2, H2_3],
    ]);
    return { Hpart: H };
};
export const hConvert = (measMatIn) => {
    // convert to hip centered origin
    let lsx = math.subset(measMatIn, math.index(0));
    let lsy = math.subset(measMatIn, math.index(1));
    let lsz = math.subset(measMatIn, math.index(2));
    let rsx = math.subset(measMatIn, math.index(3));
    let rsy = math.subset(measMatIn, math.index(4));
    let rsz = math.subset(measMatIn, math.index(5));
    let lhx = math.subset(measMatIn, math.index(6));
    let lhy = math.subset(measMatIn, math.index(7));
    let lhz = math.subset(measMatIn, math.index(8));
    let rhx = math.subset(measMatIn, math.index(9));
    let rhy = math.subset(measMatIn, math.index(10));
    let rhz = math.subset(measMatIn, math.index(11));
    let HWx = math.divide(math.add(lhx, rhx), 2);
    let HWy = math.divide(math.add(lhy, rhy), 2);
    let SWx = math.divide(math.add(lsx, rsx), 2);
    let SWy = math.divide(math.add(lsy, rsy), 2);
    let convMeas = new math.matrix([math.subtract(lsx, HWx), math.subtract(lsy, HWy), lsz, math.subtract(rsx, HWx), math.subtract(rsy, HWy), rsz, math.subtract(lhx, HWx), math.subtract(lhy, HWy), lhz, math.subtract(rhx, HWx), math.subtract(rhy, HWy), rhz]);
    return { output: convMeas, hcx: HWx, hcy: HWy, scx: SWx, scy: SWy };
};
export const HMapMeas2State_0 = (measMatIn) => {
    // convert to hip centered origin
    // for maskArray set to zeros if lsx and lsy are valid (abs()<0.5) set maskArray
    let maskArray = math.zeros(36, 1);
    let lsx = math.subset(measMatIn, math.index(0));
    let lsy = math.subset(measMatIn, math.index(1));
    let lsz = math.subset(measMatIn, math.index(2));
    if ((lsx < 1) && (lsx > 0) && (lsy < 1) && (lsy > 0)) {
        maskArray.subset(math.index(0, 0), 1);
        maskArray.subset(math.index(1, 0), 1);
        maskArray.subset(math.index(2, 0), 1);
    }
    let rsx = math.subset(measMatIn, math.index(3));
    let rsy = math.subset(measMatIn, math.index(4));
    let rsz = math.subset(measMatIn, math.index(5));
    if ((rsx < 1) && (rsx > 0) && (rsy < 1) && (rsy > 0)) {
        maskArray.subset(math.index(3, 0), 1);
        maskArray.subset(math.index(4, 0), 1);
        maskArray.subset(math.index(5, 0), 1);
    }
    let lhx = math.subset(measMatIn, math.index(6));
    let lhy = math.subset(measMatIn, math.index(7));
    let lhz = math.subset(measMatIn, math.index(8));
    if ((lhx < 1) && (lhx > 0) && (lhy < 1) && (lhy > 0)) {
        maskArray.subset(math.index(6, 0), 1);
        maskArray.subset(math.index(7, 0), 1);
        maskArray.subset(math.index(8, 0), 1);
    }
    let rhx = math.subset(measMatIn, math.index(9));
    let rhy = math.subset(measMatIn, math.index(10));
    let rhz = math.subset(measMatIn, math.index(11));
    if ((rhx < 1) && (rhx > 0) && (rhy < 1) && (rhy > 0)) {
        maskArray.subset(math.index(9, 0), 1);
        maskArray.subset(math.index(10, 0), 1);
        maskArray.subset(math.index(11, 0), 1);
    }
    let HWx = math.divide(math.add(lhx, rhx), 2);
    let HWy = math.divide(math.add(lhy, rhy), 2);
    let SWx = math.divide(math.add(lsx, rsx), 2);
    let SWy = math.divide(math.add(lsy, rsy), 2);
    let tcx = math.divide(math.add(lhx, rhx, lsx, rsx), 4);
    let tcy = math.divide(math.add(lhy, rhy, lsy, rsy), 4);
    let luax = math.subset(measMatIn, math.index(12));
    let luay = math.subset(measMatIn, math.index(13));
    let luaz = math.subset(measMatIn, math.index(14));
    if ((luax < 1) && (luax > 0) && (luay < 1) && (luay > 0)) {
        maskArray.subset(math.index(12, 0), 1);
        maskArray.subset(math.index(13, 0), 1);
        maskArray.subset(math.index(14, 0), 1);
    }
    let ruax = math.subset(measMatIn, math.index(15));
    let ruay = math.subset(measMatIn, math.index(16));
    let ruaz = math.subset(measMatIn, math.index(17));
    if ((ruax < 1) && (ruax > 0) && (ruay < 1) && (ruay > 0)) {
        maskArray.subset(math.index(15, 0), 1);
        maskArray.subset(math.index(16, 0), 1);
        maskArray.subset(math.index(17, 0), 1);
    }
    let lulx = math.subset(measMatIn, math.index(18));
    let luly = math.subset(measMatIn, math.index(19));
    let lulz = math.subset(measMatIn, math.index(20));
    if ((lulx < 1) && (lulx > 0) && (luly < 1) && (luly > 0)) {
        maskArray.subset(math.index(18, 0), 1);
        maskArray.subset(math.index(19, 0), 1);
        maskArray.subset(math.index(20, 0), 1);
    }
    let rulx = math.subset(measMatIn, math.index(21));
    let ruly = math.subset(measMatIn, math.index(22));
    let rulz = math.subset(measMatIn, math.index(23));
    if ((rulx < 1) && (rulx > 0) && (ruly < 1) && (ruly > 0)) {
        maskArray.subset(math.index(21, 0), 1);
        maskArray.subset(math.index(22, 0), 1);
        maskArray.subset(math.index(23, 0), 1);
    }
    let lwx = math.subset(measMatIn, math.index(24));
    let lwy = math.subset(measMatIn, math.index(25));
    let lwz = math.subset(measMatIn, math.index(26));
    if ((lwx < 1) && (lwx > 0) && (lwy < 1) && (lwy > 0)) {
        maskArray.subset(math.index(24, 0), 1);
        maskArray.subset(math.index(25, 0), 1);
        maskArray.subset(math.index(26, 0), 1);
    }
    let rwx = math.subset(measMatIn, math.index(27));
    let rwy = math.subset(measMatIn, math.index(28));
    let rwz = math.subset(measMatIn, math.index(29));
    if ((rwx < 1) && (rwx > 0) && (rwy < 1) && (rwy > 0)) {
        maskArray.subset(math.index(27, 0), 1);
        maskArray.subset(math.index(28, 0), 1);
        maskArray.subset(math.index(29, 0), 1);
    }
    let lax = math.subset(measMatIn, math.index(30));
    let lay = math.subset(measMatIn, math.index(31));
    let laz = math.subset(measMatIn, math.index(32));
    if ((lax < 1) && (lax > 0) && (lay < 1) && (lay > 0)) {
        maskArray.subset(math.index(30, 0), 1);
        maskArray.subset(math.index(31, 0), 1);
        maskArray.subset(math.index(32, 0), 1);
    }
    let rax = math.subset(measMatIn, math.index(33));
    let ray = math.subset(measMatIn, math.index(34));
    let raz = math.subset(measMatIn, math.index(35));
    if ((rax < 1) && (rax > 0) && (ray < 1) && (ray > 0)) {
        maskArray.subset(math.index(33, 0), 1);
        maskArray.subset(math.index(34, 0), 1);
        maskArray.subset(math.index(35, 0), 1);
    }
    let convMeas = new math.matrix([
        math.subtract(lsx, HWx), math.subtract(lsy, HWy), lsz,
        math.subtract(rsx, HWx), math.subtract(rsy, HWy), rsz,
        math.subtract(lhx, HWx), math.subtract(lhy, HWy), lhz,
        math.subtract(rhx, HWx), math.subtract(rhy, HWy), rhz,
        math.subtract(luax, HWx), math.subtract(luay, HWy), luaz,
        math.subtract(ruax, HWx), math.subtract(ruay, HWy), ruaz,
        math.subtract(lulx, HWx), math.subtract(luly, HWy), lulz,
        math.subtract(rulx, HWx), math.subtract(ruly, HWy), rulz,
        math.subtract(lwx, HWx), math.subtract(lwy, HWy), lwz,
        math.subtract(rwx, HWx), math.subtract(rwy, HWy), rwz,
        math.subtract(lax, HWx), math.subtract(lay, HWy), laz,
        math.subtract(rax, HWx), math.subtract(ray, HWy), raz
    ]);
    return { output: convMeas, hcx: HWx, hcy: HWy, scx: SWx, scy: SWy, tcx: tcx, tcy: tcy, maskArray: maskArray };
};
export const forwardModel4 = (Tquat, zin, SW, HW, L, alpha, LEquat, LE, REquat, RE, LNquat, LN, RNquat, RN, angLW, LW, angRW, RW, angLA, LA, angRA, RA) => {
    LSg.subset(math.index(0, 0), math.divide(SW, 2));
    LSg.subset(math.index(1, 0), -L);
    LSg.subset(math.index(2, 0), 0);
    RSg.subset(math.index(0, 0), math.divide(-SW, 2));
    RSg.subset(math.index(1, 0), -L);
    RSg.subset(math.index(2, 0), 0);
    LHg.subset(math.index(0, 0), math.divide(HW, 2));
    LHg.subset(math.index(1, 0), 0);
    LHg.subset(math.index(2, 0), 0);
    RHg.subset(math.index(0, 0), math.divide(-HW, 2));
    RHg.subset(math.index(1, 0), 0);
    RHg.subset(math.index(2, 0), 0);
    LEg.subset(math.index(0, 0), 0);
    LEg.subset(math.index(1, 0), LE);
    LEg.subset(math.index(2, 0), 0);
    REg.subset(math.index(0, 0), 0);
    REg.subset(math.index(1, 0), RE);
    REg.subset(math.index(2, 0), 0);
    LNg.subset(math.index(0, 0), 0);
    LNg.subset(math.index(1, 0), LN);
    LNg.subset(math.index(2, 0), 0);
    RNg.subset(math.index(0, 0), 0);
    RNg.subset(math.index(1, 0), RN);
    RNg.subset(math.index(2, 0), 0);
    LWg.subset(math.index(0, 0), 0);
    LWg.subset(math.index(1, 0), LW);
    LWg.subset(math.index(2, 0), 0);
    RWg.subset(math.index(0, 0), 0);
    RWg.subset(math.index(1, 0), RW);
    RWg.subset(math.index(2, 0), 0);
    LAg.subset(math.index(0, 0), 0);
    LAg.subset(math.index(1, 0), LA);
    LAg.subset(math.index(2, 0), 0);
    RAg.subset(math.index(0, 0), 0);
    RAg.subset(math.index(1, 0), RA);
    RAg.subset(math.index(2, 0), 0);
    let rotMat = LCGD(Tquat).LGC;
    let LSl = math.multiply(rotMat, LSg);
    let RSl = math.multiply(rotMat, RSg);
    let LHl = math.multiply(rotMat, LHg);
    let RHl = math.multiply(rotMat, RHg);
    let compLS = projection(LSl, zin, 0, 0, alpha);
    let compRS = projection(RSl, zin, 0, 0, alpha);
    let compLH = projection(LHl, zin, 0, 0, alpha);
    let compRH = projection(RHl, zin, 0, 0, alpha);
    //let rotMatTrans=math.transpose( rotMatNew );
    let rotMatLE = LCGD(LEquat).LGC;
    let rotMatRE = LCGD(REquat).LGC;
    let rotMatLN = LCGD(LNquat).LGC;
    let rotMatRN = LCGD(RNquat).LGC;
    let LEl = math.add(LSl, math.multiply(rotMat, math.multiply(rotMatLE, LEg)));
    let REl = math.add(RSl, math.multiply(rotMat, math.multiply(rotMatRE, REg)));
    let LNl = math.add(LHl, math.multiply(rotMat, math.multiply(rotMatLN, LNg)));
    let RNl = math.add(RHl, math.multiply(rotMat, math.multiply(rotMatRN, RNg)));
    let compLE = projection(LEl, zin, 0, 0, alpha);
    let compRE = projection(REl, zin, 0, 0, alpha);
    let compLN = projection(LNl, zin, 0, 0, alpha);
    let compRN = projection(RNl, zin, 0, 0, alpha);
    rotMatLW.subset(math.index(1, 1), math.cos(angLW));
    rotMatLW.subset(math.index(1, 2), math.multiply(-1, math.sin(angLW)));
    rotMatLW.subset(math.index(2, 1), math.sin(angLW));
    rotMatLW.subset(math.index(2, 2), math.cos(angLW));
    rotMatRW.subset(math.index(1, 1), math.cos(angRW));
    rotMatRW.subset(math.index(1, 2), math.multiply(-1, math.sin(angRW)));
    rotMatRW.subset(math.index(2, 1), math.sin(angRW));
    rotMatRW.subset(math.index(2, 2), math.cos(angRW));
    rotMatLA.subset(math.index(1, 1), math.cos(angLA));
    rotMatLA.subset(math.index(1, 2), math.multiply(-1, math.sin(angLA)));
    rotMatLA.subset(math.index(2, 1), math.sin(angLA));
    rotMatLA.subset(math.index(2, 2), math.cos(angLA));
    rotMatRA.subset(math.index(1, 1), math.cos(angRA));
    rotMatRA.subset(math.index(1, 2), math.multiply(-1, math.sin(angRA)));
    rotMatRA.subset(math.index(2, 1), math.sin(angRA));
    rotMatRA.subset(math.index(2, 2), math.cos(angRA));
    let LWl = math.add(LSl, math.multiply(rotMat, math.multiply(rotMatLE, math.add(LEg, math.multiply(rotMatLW, LWg)))));
    let RWl = math.add(RSl, math.multiply(rotMat, math.multiply(rotMatRE, math.add(REg, math.multiply(rotMatRW, RWg)))));
    let LAl = math.add(LHl, math.multiply(rotMat, math.multiply(rotMatLN, math.add(LNg, math.multiply(rotMatLA, LAg)))));
    let RAl = math.add(RHl, math.multiply(rotMat, math.multiply(rotMatRN, math.add(RNg, math.multiply(rotMatRA, RAg)))));
    let compLW = projection(LWl, zin, 0, 0, alpha);
    let compRW = projection(RWl, zin, 0, 0, alpha);
    let compLA = projection(LAl, zin, 0, 0, alpha);
    let compRA = projection(RAl, zin, 0, 0, alpha);
    let compMeasVec = new math.matrix([compLS.x, compLS.y, compLS.z, compRS.x, compRS.y, compRS.z,
        compLH.x, compLH.y, compLH.z, compRH.x, compRH.y, compRH.z,
        compLE.x, compLE.y, compLE.z, compRE.x, compRE.y, compRE.z,
        compLN.x, compLN.y, compLN.z, compRN.x, compRN.y, compRN.z,
        compLW.x, compLW.y, compLW.z, compRW.x, compRW.y, compRW.z,
        compLA.x, compLA.y, compLA.z, compRA.x, compRA.y, compRA.z]);
    return { output: compMeasVec };
};
export const quaternionToAxisAngle = (quat) => {
    let qax = math.subset(quat, math.index(0, 0));
    let qay = math.subset(quat, math.index(0, 1));
    let qaz = math.subset(quat, math.index(0, 2));
    let qaw = math.subset(quat, math.index(0, 3));
    let angle = math.multiply(2, math.acos(qaw));
    let scaleVal = math.sqrt(math.subtract(1, math.multiply(qaw, qaw)));
    let x = math.divide(qax, scaleVal);
    let y = math.divide(qay, scaleVal);
    let z = math.divide(qaz, scaleVal);
    // let outputQuat=new THREE.Quaternion()
    // outputQuat.setFromAxisAngle( new THREE.Vector3( x, y, z ), angle);
    return { angle: angle, x: x, y: y, z: z };
};
export const KalmanUpdateEZ = (measMatIn, tempQuatNewin, zin, SW, HW, L, alpha, Pin, deltaTime, LEquat, LE, REquat, RE, LNquat, LN, RNquat, RN, angLW, LW, angRW, RW, angLA, LA, angRA, RA, maskArray) => {
    //   let H=new math.zeros(numState,numMeas);
    let maskMat = math.multiply(maskArray, math.ones(1, numState));
    let compMeasVec = forwardModel4(tempQuatNewin, zin, SW, HW, L, alpha, LEquat, LE, REquat, RE, LNquat, LN, RNquat, RN, angLW, LW, angRW, RW, angLA, LA, angRA, RA).output;
    let residuals = math.subtract(measMatIn, compMeasVec);
    let Hmat4 = math.dotMultiply(math.transpose(maskMat), formH4(tempQuatNewin, zin, SW, HW, L, alpha, LEquat, LE, REquat, RE, LNquat, LN, RNquat, RN, angLW, angRW, angLA, angRA).output);
    // let Hmat4=formH4(tempQuatNewin, zin, SW, HW, L, alpha, LEquat, LE, REquat, RE, LNquat, LN, RNquat, RN, angLW, angRW, angLA, angRA ).output;
    let Ht = math.transpose(Hmat4);
    let Kb = null;
    let xhat = null;
    let resMat = math.matrix([residuals]);
    let PH = math.multiply(Pin, Hmat4);
    let Sb = math.add(math.multiply(Ht, math.multiply(Pin, Hmat4)), Rnew);
    let Sbi = math.inv(Sb);
    Kb = math.multiply(PH, Sbi);
    xhat = math.multiply(Kb, math.transpose(resMat));
    // 0 1 2 z sw hw L alpha 8 9 10 LE 12 13 14 RE 16 17 18 LN 20 21 22 RN
    let Phat = math.multiply(math.subtract(math.identity(numState), math.multiply(Kb, Ht)), Pin);
    let Pout = math.add(Phat, Q);
    let zOut = math.add(zin, xhat.subset(math.index(3, 0)));
    let SWout = math.add(SW, xhat.subset(math.index(4, 0)));
    let HWout = math.add(HW, xhat.subset(math.index(5, 0)));
    let Lout = math.add(L, xhat.subset(math.index(6, 0)));
    let alphaOut = math.add(alpha, xhat.subset(math.index(7, 0)));
    let tempQuat00 = quatCorrect(xhat.subset(math.index(0, 0)), xhat.subset(math.index(1, 0)), xhat.subset(math.index(2, 0)), tempQuatNewin).output;
    let LEquatOut = quatCorrect(xhat.subset(math.index(8, 0)), xhat.subset(math.index(9, 0)), xhat.subset(math.index(10, 0)), LEquat).output;
    let LEOut = math.add(LE, xhat.subset(math.index(11, 0)));
    let REquatOut = quatCorrect(xhat.subset(math.index(12, 0)), xhat.subset(math.index(13, 0)), xhat.subset(math.index(14, 0)), REquat).output;
    let REOut = math.add(RE, xhat.subset(math.index(15, 0)));
    let LNquatOut = quatCorrect(xhat.subset(math.index(16, 0)), xhat.subset(math.index(17, 0)), xhat.subset(math.index(18, 0)), LNquat).output;
    let LNOut = math.add(LN, xhat.subset(math.index(19, 0)));
    let RNquatOut = quatCorrect(xhat.subset(math.index(20, 0)), xhat.subset(math.index(21, 0)), xhat.subset(math.index(22, 0)), RNquat).output;
    let RNOut = math.add(RN, xhat.subset(math.index(23, 0)));
    //angLW, LW, angRW, RW, angLA, LA, angRA, RA
    let angLWOut = math.add(angLW, xhat.subset(math.index(24, 0)));
    let LWOut = math.add(LW, xhat.subset(math.index(25, 0)));
    let angRWOut = math.add(angRW, xhat.subset(math.index(26, 0)));
    let RWOut = math.add(RW, xhat.subset(math.index(27, 0)));
    let angLAOut = math.add(angLA, xhat.subset(math.index(28, 0)));
    let LAOut = math.add(LA, xhat.subset(math.index(29, 0)));
    let angRAOut = math.add(angRA, xhat.subset(math.index(30, 0)));
    let RAOut = math.add(RA, xhat.subset(math.index(31, 0)));
    let logOn = 0;
    if (logOn == 1) {
        console.log(' statLog ' + math.subset(tempQuatNewin, math.index(0, 0)) + ' ' + math.subset(tempQuatNewin, math.index(0, 1)) + ' ' + math.subset(tempQuatNewin, math.index(0, 2)) + ' ' + math.subset(tempQuatNewin, math.index(0, 3)) + ' ' + math.subset(measMatIn, math.index(0)) + ' ' + math.subset(measMatIn, math.index(1)) + ' ' + math.subset(measMatIn, math.index(2)) + ' ' + math.subset(measMatIn, math.index(3)) + ' '
            + math.subset(measMatIn, math.index(4)) + ' ' + math.subset(measMatIn, math.index(5)) + ' ' + math.subset(measMatIn, math.index(6)) + ' ' + math.subset(measMatIn, math.index(7)) + ' ' + math.subset(measMatIn, math.index(8)) + ' ' + math.subset(measMatIn, math.index(9)) + ' ' + math.subset(measMatIn, math.index(10)) + ' ' + math.subset(measMatIn, math.index(11)) + ' '
            + math.subset(measMatIn, math.index(12)) + ' ' + math.subset(measMatIn, math.index(13)) + ' ' + math.subset(measMatIn, math.index(14)) + ' ' + math.subset(measMatIn, math.index(15)) + ' ' + math.subset(measMatIn, math.index(16)) + ' ' + math.subset(measMatIn, math.index(17)) + ' ' + math.subset(measMatIn, math.index(18)) + ' ' + math.subset(measMatIn, math.index(19)) + ' '
            + math.subset(measMatIn, math.index(20)) + ' ' + math.subset(measMatIn, math.index(21)) + ' ' + math.subset(measMatIn, math.index(22)) + ' ' + math.subset(measMatIn, math.index(23)) + ' '
            + math.subset(measMatIn, math.index(24)) + ' ' + math.subset(measMatIn, math.index(25)) + ' ' + math.subset(measMatIn, math.index(26)) + ' '
            + math.subset(measMatIn, math.index(27)) + ' ' + math.subset(measMatIn, math.index(28)) + ' ' + math.subset(measMatIn, math.index(29)) + ' '
            + math.subset(measMatIn, math.index(30)) + ' ' + math.subset(measMatIn, math.index(31)) + ' ' + math.subset(measMatIn, math.index(32)) + ' '
            + math.subset(measMatIn, math.index(33)) + ' ' + math.subset(measMatIn, math.index(34)) + ' ' + math.subset(measMatIn, math.index(35)) + ' '
            + zin + ' ' + SW + ' ' + HW + ' ' + L + ' ' + alpha + ' ' + deltaTime + ' '
            + math.subset(LEquat, math.index(0, 0)) + ' ' + math.subset(LEquat, math.index(0, 1)) + ' ' + math.subset(LEquat, math.index(0, 2)) + ' ' + math.subset(LEquat, math.index(0, 3)) + ' ' + LE + ' '
            + math.subset(REquat, math.index(0, 0)) + ' ' + math.subset(REquat, math.index(0, 1)) + ' ' + math.subset(REquat, math.index(0, 2)) + ' ' + math.subset(REquat, math.index(0, 3)) + ' ' + RE + ' '
            + math.subset(LNquat, math.index(0, 0)) + ' ' + math.subset(LNquat, math.index(0, 1)) + ' ' + math.subset(LNquat, math.index(0, 2)) + ' ' + math.subset(LNquat, math.index(0, 3)) + ' ' + LN + ' '
            + math.subset(RNquat, math.index(0, 0)) + ' ' + math.subset(RNquat, math.index(0, 1)) + ' ' + math.subset(RNquat, math.index(0, 2)) + ' ' + math.subset(RNquat, math.index(0, 3)) + ' ' + RN + ' '
            + angLW + ' ' + LW + ' ' + angRW + ' ' + RW + ' ' + angLA + ' ' + LA + ' ' + angRA + ' ' + RA + ' '
            + xhat.subset(math.index(0, 0)) + ' ' + xhat.subset(math.index(1, 0)) + ' ' + xhat.subset(math.index(2, 0)) + ' ' + xhat.subset(math.index(3, 0)) + ' '
            + xhat.subset(math.index(4, 0)) + ' ' + xhat.subset(math.index(5, 0)) + ' ' + xhat.subset(math.index(6, 0)) + ' ' + xhat.subset(math.index(7, 0)) + ' '
            + xhat.subset(math.index(8, 0)) + ' ' + xhat.subset(math.index(9, 0)) + ' ' + xhat.subset(math.index(10, 0)) + ' ' + xhat.subset(math.index(11, 0)) + ' ');
    }
    return { quat: tempQuat00, z: zOut, SW: SWout, HW: HWout, L: Lout, alpha: alphaOut, cmeas: compMeasVec,
        P: Pout, LEquat: LEquatOut, LE: LEOut, REquat: REquatOut, RE: REOut, LNquat: LNquatOut, LN: LNOut, RNquat: RNquatOut, RN: RNOut,
        angLW: angLWOut, LW: LWOut, angRW: angRWOut, RW: RWOut, angLA: angLAOut, LA: LAOut, angRA: angRAOut, RA: RAOut };
};
function checkBindex(indexIn) {
    // let L=indexIn.length;
    // alert disable with: let output=true;
    let output = true;
    let sum = 0;
    for (let i = 0; i < 4; i++) {
        sum = sum + math.subset(indexIn, math.index(0, i));
    }
    if (sum >= 3.9) {
        output = true;
    }
    return { output: output };
}
export const KalmanUpdateB = (measMatIn, tempQuatNewin, zin, SW, HW, L, alpha, Pin, deltaTime, LEquat, LE, REquat, RE, LNquat, LN, RNquat, RN, angLW, LW, angRW, RW, angLA, LA, angRA, RA) => {
    //  debugger;
    let compMeasVec = forwardModel4(tempQuatNewin, zin, SW, HW, L, alpha, LEquat, LE, REquat, RE, LNquat, LN, RNquat, RN, angLW, LW, angRW, RW, angLA, LA, angRA, RA).output;
    let residuals = math.subtract(measMatIn, compMeasVec);
    let Hmat4 = formH4(tempQuatNewin, zin, SW, HW, L, alpha, LEquat, LE, REquat, RE, LNquat, LN, RNquat, RN, angLW, angRW, angLA, angRA).output;
    let Ht = math.transpose(Hmat4);
    //  epp count alpha beta hardcode LW RW LA RA
    let count = 1;
    let epp = 1;
    let bIndices = new math.matrix([24, 26, 28, 30]);
    let alphaP = new math.matrix([math.pi, math.pi, 0, 0]);
    let betaP = new math.matrix([0, 0, -math.pi, -math.pi]);
    let stateP = new math.matrix([angLW, angRW, angLA, angRA]);
    // for loop make ce_mat
    let CE = math.zeros(32, 32);
    let bIndex = math.zeros(1, 4);
    let xbar = math.zeros(32, 1);
    let xDelta = math.zeros(32, 1);
    // for (bIndices(k), bIndices(k)) set CE
    // to get (math.subset(indexIn, math.index(0,i)))
    for (let i = 0; i < 4; i++) {
        let cIndex = (math.subset(bIndices, math.index(i)));
        let betaVal = (math.subset(betaP, math.index(i)));
        let alphaVal = (math.subset(alphaP, math.index(i)));
        let stateVal = (math.subset(stateP, math.index(i)));
        let cVal = math.divide(2, math.pow(math.subtract(betaVal, alphaVal), 2));
        CE.subset(math.index(cIndex, cIndex), cVal);
        let mVal = math.divide(math.add(betaVal, alphaVal), 2);
        xbar.subset(math.index(cIndex, 0), mVal);
        xDelta.subset(math.index(cIndex, 0), math.subtract(mVal, stateVal));
    }
    let Kb = null;
    let xhat = null;
    let resMat = math.matrix([residuals]);
    let PH = math.multiply(Pin, Hmat4);
    //  alert and bIndex
    //  let S0=math.multiply(Ht, PH);
    //  let S=math.add(S0,Rnew);
    //  let Si=math.inv(S);
    //  let Kgain=math.multiply(PH, Si);
    let Sb = math.add(math.multiply(Ht, math.multiply(Pin, Hmat4)), Rnew);
    let Sbi = math.inv(Sb);
    Kb = math.multiply(PH, Sbi);
    xhat = math.multiply(Kb, math.transpose(resMat));
    while (checkBindex(bIndex).output == false) {
        //let PH=math.multiply(math.add(Pin,CE), Hmat4);
        let Sb = math.add(math.multiply(Ht, math.multiply(math.add(Pin, CE), Hmat4)), Rnew);
        let Sbi = math.inv(Sb);
        Kb = math.multiply(PH, Sbi);
        let CExDelta = math.multiply(CE, xDelta);
        let HtCExDelta = math.multiply(Ht, CExDelta);
        let resMatBar = math.add(resMat, math.transpose(HtCExDelta));
        xhat = math.multiply(Kb, math.transpose(resMatBar));
        //alt xhat=math.multiply(Kb,math.add(residuals, math.multiply(Ht,CE,xDelta )));
        for (let i = 0; i < 4; i++) {
            let cIndex = (math.subset(bIndices, math.index(i)));
            let betaVal = (math.subset(betaP, math.index(i)));
            let alphaVal = (math.subset(alphaP, math.index(i)));
            let stateVal = (math.subset(stateP, math.index(i)));
            let testVal = math.add(stateVal, (math.subset(xhat, math.index(cIndex, 0))));
            if ((testVal < alphaVal) & (testVal > betaVal)) {
                bIndex.subset(math.index(0, i), 1);
            }
            else {
                let ceVal = math.subset(CE, math.index(cIndex, cIndex));
                let ceValNew = math.multiply(ceVal, math.add(1, math.divide(count, 5)));
                CE.subset(math.index(cIndex, cIndex), ceValNew);
            }
        }
        count = count + 1;
    }
    // 0 1 2 z sw hw L alpha 8 9 10 LE 12 13 14 RE 16 17 18 LN 20 21 22 RN
    let Phat = math.multiply(math.subtract(math.identity(numState), math.multiply(Kb, Ht)), Pin);
    let Pout = math.add(Phat, Q);
    let zOut = math.add(zin, xhat.subset(math.index(3, 0)));
    let SWout = math.add(SW, xhat.subset(math.index(4, 0)));
    let HWout = math.add(HW, xhat.subset(math.index(5, 0)));
    let Lout = math.add(L, xhat.subset(math.index(6, 0)));
    let alphaOut = math.add(alpha, xhat.subset(math.index(7, 0)));
    let tempQuat00 = quatCorrect(xhat.subset(math.index(0, 0)), xhat.subset(math.index(1, 0)), xhat.subset(math.index(2, 0)), tempQuatNewin).output;
    let LEquatOut = quatCorrect(xhat.subset(math.index(8, 0)), xhat.subset(math.index(9, 0)), xhat.subset(math.index(10, 0)), LEquat).output;
    let LEOut = math.add(LE, xhat.subset(math.index(11, 0)));
    let REquatOut = quatCorrect(xhat.subset(math.index(12, 0)), xhat.subset(math.index(13, 0)), xhat.subset(math.index(14, 0)), REquat).output;
    let REOut = math.add(RE, xhat.subset(math.index(15, 0)));
    let LNquatOut = quatCorrect(xhat.subset(math.index(16, 0)), xhat.subset(math.index(17, 0)), xhat.subset(math.index(18, 0)), LNquat).output;
    let LNOut = math.add(LN, xhat.subset(math.index(19, 0)));
    let RNquatOut = quatCorrect(xhat.subset(math.index(20, 0)), xhat.subset(math.index(21, 0)), xhat.subset(math.index(22, 0)), RNquat).output;
    let RNOut = math.add(RN, xhat.subset(math.index(23, 0)));
    //angLW, LW, angRW, RW, angLA, LA, angRA, RA
    let angLWOut = math.add(angLW, xhat.subset(math.index(24, 0)));
    let LWOut = math.add(LW, xhat.subset(math.index(25, 0)));
    let angRWOut = math.add(angRW, xhat.subset(math.index(26, 0)));
    let RWOut = math.add(RW, xhat.subset(math.index(27, 0)));
    let angLAOut = math.add(angLA, xhat.subset(math.index(28, 0)));
    let LAOut = math.add(LA, xhat.subset(math.index(29, 0)));
    let angRAOut = math.add(angRA, xhat.subset(math.index(30, 0)));
    let RAOut = math.add(RA, xhat.subset(math.index(31, 0)));
    return { quat: tempQuat00, z: zOut, SW: SWout, HW: HWout, L: Lout, alpha: alphaOut, cmeas: compMeasVec,
        P: Pout, LEquat: LEquatOut, LE: LEOut, REquat: REquatOut, RE: REOut, LNquat: LNquatOut, LN: LNOut, RNquat: RNquatOut, RN: RNOut,
        angLW: angLWOut, LW: LWOut, angRW: angRWOut, RW: RWOut, angLA: angLAOut, LA: LAOut, angRA: angRAOut, RA: RAOut };
};
export function multiDimensionalUnique(arr) {
    var uniques = [];
    var itemsFound = {};
    for (var i = 0, l = arr.length; i < l; i++) {
        var stringified = JSON.stringify(arr[i]);
        if (itemsFound[stringified]) {
            continue;
        }
        uniques.push(arr[i]);
        itemsFound[stringified] = true;
    }
    return uniques;
}
export function traversePolygon(polygons, callback) {
    polygons.forEach(polygon => {
        polygon.forEach(point => {
            callback(point);
        });
    });
}
export function traverseMultiPolygon(multiPolygons, callback) {
    multiPolygons.forEach(multiPolygon => {
        multiPolygon.forEach(polygon => {
            polygon.forEach(point => {
                callback(point);
            });
        });
    });
}
export function createGridPoints(data, ballSpaceX, ballSpaceY, ppDz) {
    const districtData = data.features[0];
    // 1.1 Caculate the extent of your shape coordinate
    const extent = turf.bbox(districtData);
    const minX = Math.trunc(extent[0]);
    const maxX = Math.trunc(extent[2]);
    const minY = Math.trunc(extent[1]);
    const maxY = Math.trunc(extent[3]);
    let gridPoints = [];
    let clothWidth = maxX - minX;
    let clothHeight = maxY - minY;
    let stepSizeX = clothWidth / (ballSpaceX - 1);
    let stepSizeY = clothHeight / (ballSpaceY - 1);
    const nBinsX = math.ceil(ballSpaceX);
    const histogramX = new Array(nBinsX).fill(0);
    // const indexX = [];//new Array(binsX).fill(0);
    const nBinsY = math.ceil(ballSpaceY);
    const histogramY = new Array(nBinsY).fill(0);
    // const indexY = [];//new Array(binsX).fill(0);
    const histogramXvals = new Array(nBinsX).fill(0);
    // for (const item of data) {
    //     histogram[Math.floor((item - minX) / stepSizeX)]++;
    // }
    // for(let i = 0; i < (ballSpaceX+1); i++) {
    //   let ballPosX=minX+stepSizeX*i -stepSizeX/2;
    //   indexX.push(ballPosX)
    // }
    // for(let i = 0; i < (ballSpaceY+1); i++) {
    //   let ballPosY=minY+stepSizeY*i - stepSizeY/2;
    //   indexY.push(ballPosY)
    // }
    // 1.2 Create an array of coordinates with a  fixed intervals in x and y direction.
    for (let i = 0; i < (ballSpaceX + 1); i++) {
        for (let j = 0; j < (ballSpaceY + 1); j++) {
            let ballPosX = minX + stepSizeX * i - stepSizeX / 2;
            let ballPosY = minY + stepSizeY * j - stepSizeY / 2;
            gridPoints.push([ballPosX, ballPosY]);
        }
    }
    gridPoints = turf.points(gridPoints);
    const points3d = [];
    // 2.1 Finds Points that fall within shape(polygon)
    const inPoint = turf.pointsWithinPolygon(gridPoints, districtData).features;
    gridPointLength = inPoint.length;
    // get all x and y vals then unique and sort
    //
    const xValsRaw = [];
    const yValsRaw = [];
    inPoint.forEach((point, ii) => {
        const coordinates = point.geometry.coordinates;
        xValsRaw.push(coordinates[0]);
        yValsRaw.push(coordinates[1]);
        points3d.push(new THREE.Vector3(coordinates[0], coordinates[1], ppDz));
        //TODO not working correctly for hist8 should be 8 8 ... but get 16 0 ...
        histogramX[math.floor(math.divide(math.subtract(coordinates[0], minX), stepSizeX))]++;
        histogramY[math.floor(math.divide(math.subtract(coordinates[1], minY), stepSizeY))]++;
    });
    let indexX = multiDimensionalUnique(xValsRaw).sort((a, b) => a - b);
    let indexY = multiDimensionalUnique(yValsRaw).sort((a, b) => a - b);
    let minX0 = indexX[0];
    let minY0 = indexY[0];
    let linkMap = Array(indexX.length + 2).fill().map(() => Array(indexY.length + 2).fill(0));
    inPoint.forEach((point, ii) => {
        const coordinates = point.geometry.coordinates;
        let xcoord = math.floor(math.divide(math.subtract(coordinates[0], minX), stepSizeX)) + 1;
        let ycoord = math.floor(math.divide(math.subtract(coordinates[1], minY), stepSizeY)) + 1;
        linkMap[xcoord][ycoord] = ii;
    });
    let links = [];
    for (let i = 0; i < (indexX.length + 2); i++) {
        for (let j = 0; j < (indexY.length + 2); j++) {
            if (linkMap[i][j] != 0) {
                if (linkMap[i][j + 1] != 0) {
                    links.push([linkMap[i][j], linkMap[i][j + 1]]);
                }
                if (linkMap[i + 1][j + 1] != 0) {
                    links.push([linkMap[i][j], linkMap[i + 1][j + 1]]);
                }
                if (linkMap[i + 1][j] != 0) {
                    links.push([linkMap[i][j], linkMap[i + 1][j]]);
                }
                if (linkMap[i + 1][j - 1] != 0) {
                    links.push([linkMap[i][j], linkMap[i + 1][j - 1]]);
                }
                if (linkMap[i][j - 1] != 0) {
                    links.push([linkMap[i][j], linkMap[i][j - 1]]);
                }
                if (linkMap[i - 1][j - 1] != 0) {
                    links.push([linkMap[i][j], linkMap[i - 1][j - 1]]);
                }
                if (linkMap[i - 1][j] != 0) {
                    links.push([linkMap[i][j], linkMap[i - 1][j]]);
                }
                if (linkMap[i - 1][j + 1] != 0) {
                    links.push([linkMap[i][j], linkMap[i - 1][j + 1]]);
                }
            }
        }
    }
    // 2.2 Push all shape points to the points array.
    switch (districtData.geometry.type) {
        case "Broken?Polygon":
            traversePolygon(districtData.geometry.coordinates, point => {
                points3d.push(new THREE.Vector3(point[0], point[1], 0));
            });
            break;
        case "MultiPolygon":
            traverseMultiPolygon(districtData.geometry.coordinates, point => {
                points3d.push(new THREE.Vector3(point[0], point[1], 0));
            });
            break;
    }
    return { points3d: points3d, histogramX: histogramX, indexX: indexX, histogramY: histogramY, indexY: indexY, links: links };
    // return points3d;
}
export function createGeometry(points3d, maxTriangleSize) {
    // 3. Caculate the faces indices array of the points array by using Delaunator.js
    const indexDelaunay = Delaunator.from(points3d.map(v => {
        return [v.x, v.y];
    }));
    const meshIndex = [];
    for (let i = 0; i < indexDelaunay.triangles.length; i++) {
        meshIndex.push(indexDelaunay.triangles[i]);
    }
    // 4. Delete the unwanted indices
    let filterdMeshIndex = [];
    // 5. TODO max triangle size shouldnt be hard coded, depends on scale, nBalls
    //  necessary to remove atifacts- unwanted connections
    // let maxTriangleSize=8;
    for (let i = 0; i < meshIndex.length; i += 3) {
        let posA = points3d[meshIndex[i]];
        let posB = points3d[meshIndex[i + 1]];
        let posC = points3d[meshIndex[i + 2]];
        let dAB = math.sqrt(math.pow(posA.x - posB.x, 2) + math.pow(posA.y - posB.y, 2) + math.pow(posA.z - posB.z, 2));
        let dBC = math.sqrt(math.pow(posC.x - posB.x, 2) + math.pow(posC.y - posB.y, 2) + math.pow(posC.z - posB.z, 2));
        let dAC = math.sqrt(math.pow(posA.x - posC.x, 2) + math.pow(posA.y - posC.y, 2) + math.pow(posA.z - posC.z, 2));
        if (dAB < maxTriangleSize && dAC < maxTriangleSize && dBC < maxTriangleSize) {
            filterdMeshIndex.push([meshIndex[i], meshIndex[i + 1], meshIndex[i + 2]]);
        }
    }
    filterdMeshIndex = filterdMeshIndex.filter(item => {
        const minItem = math.min(...item);
        let flag = true;
        if (((item[0] > item[1] && item[1] > item[2]) || (item[1] > item[2] && item[2] > item[0]) || (item[2] > item[0] && item[0] > item[1])) && minItem > gridPointLength) {
            flag = false;
        }
        return flag;
    });
    filterdMeshIndex = filterdMeshIndex.flat(1);
    //  console.log(filterdMeshIndex);
    // 5. Creat buffergeometry with the points array and indices array.
    const geometry = new THREE.BufferGeometry().setFromPoints(points3d);
    geometry.setIndex(filterdMeshIndex);
    geometry.computeVertexNormals();
    return { geometry: geometry, points: points3d, indices: filterdMeshIndex };
}
// const initRef= useRef(false);
let tempQuatNew = math.zeros(1, 4); // 
let bodyQuatOffset = math.zeros(1, 4); // 
let quatLE = math.zeros(1, 4);
let quatRE = math.zeros(1, 4);
let quatLN = math.zeros(1, 4);
let quatRN = math.zeros(1, 4);
let dTemp = 0.5;
let dTempS = 0.5;
let armRenderWidth = 0.06;
let alphaSF = -1; // = 1/6.7;
// initialize all body lengths in meters
let SW = 0.37;
let HW = 0.32;
let L = 0.49;
let LE = 0.3;
let RE = 0.3;
let LN = 0.46;
let RN = 0.46;
let LW = 0.27;
let RW = 0.27;
let RA = 0.45;
let LA = 0.45;
// initial uncertainty values for Kalman
let PvalTheta = 0.5;
let Pvalz = 0.1;
let PvalSW = 0.0;
let PvalHW = 0.01;
let PvalL = 0.01;
let PvalAlpha = 0.0;
let PvalLE = 0.01;
let PvalRE = 0.01;
let PvalLN = 0.01;
let PvalRN = 0.01;
let PvalLW = 0.01;
let PvalRW = 0.01;
let PvalLA = 0.01;
let PvalRA = 0.01;
let Qtheta = 0.5;
let Qtheta3 = 0.5;
let Qz = 0.1;
let Qdims = 0.001;
let QdimsSW = 0;
let Qalpha = 0;
let Q = math.identity(numState);
Q = math.multiply(Q, Qtheta);
let Qt1 = 0.5; // bow?
let Qt2 = 0.5; // twist
let Qt3 = 0.5; // roll
Q.subset(math.index(0, 0), Qt1);
Q.subset(math.index(1, 1), Qt2);
Q.subset(math.index(2, 2), Qt3);
Q.subset(math.index(3, 3), Qz);
Q.subset(math.index(4, 4), QdimsSW);
Q.subset(math.index(5, 5), Qdims);
Q.subset(math.index(6, 6), Qdims);
Q.subset(math.index(7, 7), Qalpha); //8-10
Q.subset(math.index(10, 10), Qtheta3);
Q.subset(math.index(11, 11), Qdims); // 12-14
Q.subset(math.index(14, 14), Qtheta3);
Q.subset(math.index(15, 15), Qdims); // 16-18
Q.subset(math.index(18, 18), Qtheta3);
Q.subset(math.index(19, 19), Qdims); //20-22
Q.subset(math.index(22, 22), Qtheta3);
Q.subset(math.index(23, 23), Qdims);
Q.subset(math.index(25, 25), Qdims);
Q.subset(math.index(27, 27), Qdims);
Q.subset(math.index(29, 29), Qdims);
Q.subset(math.index(31, 31), Qdims);
let angLW = math.multiply(0, math.pi);
let angRW = math.multiply(0, math.pi);
let angLA = math.multiply(0, math.pi);
let angRA = math.multiply(0, math.pi);
// P is state uncertainty matrix for Kalman
let P = math.identity(32);
P = math.multiply(PvalTheta, P);
P.subset(math.index(3, 3), Pvalz);
P.subset(math.index(4, 4), PvalSW);
P.subset(math.index(5, 5), PvalHW);
P.subset(math.index(6, 6), PvalL);
P.subset(math.index(7, 7), PvalAlpha);
P.subset(math.index(11, 11), PvalLE);
P.subset(math.index(15, 15), PvalRE);
P.subset(math.index(19, 19), PvalLN);
P.subset(math.index(23, 23), PvalRN);
P.subset(math.index(25, 25), PvalLW);
P.subset(math.index(27, 27), PvalRW);
P.subset(math.index(29, 29), PvalLA);
P.subset(math.index(31, 31), PvalRA);
let currTimeSec = 0;
let lastTimeSec = 0;
let initRef = false;
export function passData(poseLandmarks, currTimeSec, segMask, bodyScale) {
    // console.log(" passData ")
    let numInit = 1;
    // let numInit0=1;
    let shoulderLx = poseLandmarks[11].x;
    let shoulderLy = poseLandmarks[11].y;
    let shoulderLz = poseLandmarks[11].z;
    let shoulderRx = poseLandmarks[12].x;
    let shoulderRy = poseLandmarks[12].y;
    let shoulderRz = poseLandmarks[12].z;
    let hipLx = poseLandmarks[23].x;
    let hipLy = poseLandmarks[23].y;
    let hipLz = poseLandmarks[23].z;
    let hipRx = poseLandmarks[24].x;
    let hipRy = poseLandmarks[24].y;
    let hipRz = poseLandmarks[24].z;
    let armUpperLx = poseLandmarks[13].x;
    let armUpperLy = poseLandmarks[13].y;
    let armUpperLz = poseLandmarks[13].z;
    let armUpperRx = poseLandmarks[14].x;
    let armUpperRy = poseLandmarks[14].y;
    let armUpperRz = poseLandmarks[14].z;
    let legUpperLx = poseLandmarks[25].x;
    let legUpperLy = poseLandmarks[25].y;
    let legUpperLz = poseLandmarks[25].z;
    let legUpperRx = poseLandmarks[26].x;
    let legUpperRy = poseLandmarks[26].y;
    let legUpperRz = poseLandmarks[26].z;
    let wristLx = poseLandmarks[15].x;
    let wristLy = poseLandmarks[15].y;
    let wristLz = poseLandmarks[15].z;
    let wristRx = poseLandmarks[16].x;
    let wristRy = poseLandmarks[16].y;
    let wristRz = poseLandmarks[16].z;
    let ankleLx = poseLandmarks[27].x;
    let ankleLy = poseLandmarks[27].y;
    let ankleLz = poseLandmarks[27].z;
    let ankleRx = poseLandmarks[28].x;
    let ankleRy = poseLandmarks[28].y;
    let ankleRz = poseLandmarks[28].z;
    // use predictiveIfAvailable
    let conversionOutput = HMapMeas2State_0(math.matrix([shoulderLx, shoulderLy, shoulderLz, shoulderRx, shoulderRy, shoulderRz,
        hipLx, hipLy, hipLz, hipRx, hipRy, hipRz, armUpperLx, armUpperLy, armUpperLz, armUpperRx, armUpperRy, armUpperRz,
        legUpperLx, legUpperLy, legUpperLz, legUpperRx, legUpperRy, legUpperRz, wristLx, wristLy, wristLz, wristRx,
        wristRy, wristRz, ankleLx, ankleLy, ankleLz, ankleRx, ankleRy, ankleRz]));
    let measMatFlap = conversionOutput.output;
    let hcx = conversionOutput.hcx; // same
    let hcy = conversionOutput.hcy;
    let scx = conversionOutput.scx; // same
    let scy = conversionOutput.scy;
    let tcx = conversionOutput.tcx; // same
    let tcy = conversionOutput.tcy;
    // let temp_compMeasVec= computedPoints;
    let maskArray = conversionOutput.maskArray;
    // TODO need better entry condition stability of points or skeleton and size threshold; all points available?
    if (!initRef) {
        // reset causes all lengths to reset
        let SW = 0.37;
        let HW = 0.32;
        let L = 0.49;
        let LE = 0.3;
        let RE = 0.3;
        let LN = 0.46;
        let RN = 0.46;
        let LW = 0.27;
        let RW = 0.27;
        let RA = 0.45;
        let LA = 0.45;
        //  let itheta1L=math.divide(math.pi,-2)
        // let = math.matrix([[0, 0, math.sin(math.divide(itheta1L,2)), math.cos(math.divide(itheta1L,2))]])
        tempQuatNew.subset(math.index(0, 0), 0);
        tempQuatNew.subset(math.index(0, 1), 0);
        tempQuatNew.subset(math.index(0, 2), 0);
        tempQuatNew.subset(math.index(0, 3), 1);
        //-0.3835   -0.4845    0.5249    0.5854
        quatLE.subset(math.index(0, 0), -0.3835);
        quatLE.subset(math.index(0, 1), -0.4845);
        quatLE.subset(math.index(0, 2), 0.5249);
        quatLE.subset(math.index(0, 3), 0.5854);
        //    -0.2119    0.2321   -0.6862    0.6560
        quatRE.subset(math.index(0, 0), -0.2119);
        quatRE.subset(math.index(0, 1), 0.2321);
        quatRE.subset(math.index(0, 2), -0.6862);
        quatRE.subset(math.index(0, 3), 0.6560);
        quatLN.subset(math.index(0, 0), 0);
        quatLN.subset(math.index(0, 1), 0);
        quatLN.subset(math.index(0, 2), 0);
        quatLN.subset(math.index(0, 3), 1);
        quatRN.subset(math.index(0, 0), 0);
        quatRN.subset(math.index(0, 1), 0);
        quatRN.subset(math.index(0, 2), 0);
        quatRN.subset(math.index(0, 3), 1);
        // angLW angRW
        angLW = 0.33;
        angRW = 0.24;
        alphaSF = -1; // = 1/6.7;
        P = math.identity(32);
        P = math.multiply(PvalTheta, P);
        P.subset(math.index(3, 3), Pvalz);
        P.subset(math.index(4, 4), PvalSW);
        P.subset(math.index(5, 5), PvalHW);
        P.subset(math.index(6, 6), PvalL);
        P.subset(math.index(7, 7), PvalAlpha);
        P.subset(math.index(11, 11), PvalLE);
        P.subset(math.index(15, 15), PvalRE);
        P.subset(math.index(19, 19), PvalLN);
        P.subset(math.index(23, 23), PvalRN);
        P.subset(math.index(25, 25), PvalLW);
        P.subset(math.index(27, 27), PvalRW);
        P.subset(math.index(29, 29), PvalLA);
        P.subset(math.index(31, 31), PvalRA);
        let kalmanOutput = KalmanUpdateEZ(measMatFlap, tempQuatNew, dTemp, SW, HW, L, alphaSF, P, 0, quatLE, LE, quatRE, RE, quatLN, LN, quatRN, RN, angLW, LW, angRW, RW, angLA, LA, angRA, RA, maskArray);
        //let kalmanOutput=KalmanUpdateF4(measMatFlap,  tempQuatNew, dTemp, SW, HW, L, alphaSF, P, 0);
        tempQuatNew = kalmanOutput.quat;
        dTemp = kalmanOutput.z;
        SW = kalmanOutput.SW;
        HW = kalmanOutput.HW;
        L = kalmanOutput.L;
        alphaSF = kalmanOutput.alpha;
        quatLE = kalmanOutput.LEquat;
        LE = kalmanOutput.LE;
        quatRE = kalmanOutput.REquat;
        RE = kalmanOutput.RE;
        quatLN = kalmanOutput.LNquat;
        LN = kalmanOutput.LN;
        quatRN = kalmanOutput.RNquat;
        RN = kalmanOutput.RN;
        angLW = kalmanOutput.angLW;
        LW = kalmanOutput.LW;
        angRW = kalmanOutput.angRW;
        RW = kalmanOutput.RW;
        angLA = kalmanOutput.angLA;
        LA = kalmanOutput.LA;
        angRA = kalmanOutput.angRA;
        RA = kalmanOutput.RA;
        // for (let j = 0; j < numQuatAvg; j++) {
        //   bodyQuatNewA.current[j]=math.matrix([[tempQuatNew.toArray()[0][0], -tempQuatNew.toArray()[0][1], tempQuatNew.toArray()[0][2], tempQuatNew.toArray()[0][3]]])
        //   }
        lastTimeSec = currTimeSec;
        initRef = true;
        // stitchLength.current=stitchStartLength
        // TODO force re-render programtically with useState
        // setMaleFemale(true)
        // initialized=true;
        // currInitialized=true;
    }
    let deltaTimeSec = math.divide(math.subtract(currTimeSec, lastTimeSec), 1000000);
    currTimeSec = lastTimeSec;
    let cmeasH0 = null;
    for (let i = 0; i < numInit; i++) {
        let kalmanOutput = KalmanUpdateEZ(measMatFlap, tempQuatNew, dTemp, SW, HW, L, alphaSF, P, 0, quatLE, LE, quatRE, RE, quatLN, LN, quatRN, RN, angLW, LW, angRW, RW, angLA, LA, angRA, RA, maskArray);
        tempQuatNew = kalmanOutput.quat;
        bodyQuatOffset = math.matrix([[tempQuatNew.toArray()[0][0], -tempQuatNew.toArray()[0][1], tempQuatNew.toArray()[0][2], tempQuatNew.toArray()[0][3]]]);
        // bodyQuatNew2.current=bodyQuatNewOffset;
        //TODO
        // bodyQuatNewA.current[countPasses.current % numQuatAvg]=math.matrix([[tempQuatNew.toArray()[0][0], -tempQuatNew.toArray()[0][1], tempQuatNew.toArray()[0][2], tempQuatNew.toArray()[0][3]]])
        //  smoother writes mod window of last 3 to 5 vals
        //     bodyQuatNewA.current[countPasses.current % numQuatAvg]=math.matrix([[tempQuatNew.toArray()[0][0], -tempQuatNew.toArray()[0][1], tempQuatNew.toArray()[0][2], tempQuatNew.toArray()[0][3]]])
        // average it
        dTemp = kalmanOutput.z;
        SW = kalmanOutput.SW;
        HW = kalmanOutput.HW;
        L = kalmanOutput.L;
        alphaSF = kalmanOutput.alpha;
        P = kalmanOutput.P;
        quatLE = kalmanOutput.LEquat;
        LE = kalmanOutput.LE;
        quatRE = kalmanOutput.REquat;
        RE = kalmanOutput.RE;
        quatLN = kalmanOutput.LNquat;
        LN = kalmanOutput.LN;
        quatRN = kalmanOutput.RNquat;
        RN = kalmanOutput.RN;
        angLW = kalmanOutput.angLW;
        LW = kalmanOutput.LW;
        angRW = kalmanOutput.angRW;
        RW = kalmanOutput.RW;
        angLA = kalmanOutput.angLA;
        LA = kalmanOutput.LA;
        angRA = kalmanOutput.angRA;
        RA = kalmanOutput.RA;
        cmeasH0 = kalmanOutput.cmeas;
    }
    dTempS = math.multiply(bodyScale, dTemp);
    // all this uses gl projection for scaling, so translation lines up
    // TODO clean up
    // let relCamZposition=math.multiply(-1,math.sqrt(math.multiply(dTempS,dTempS)));
    // let scaledZposition=math.add(math.multiply(proj10.current,dTempS),proj14.current)
    // let desiredXposition=math.subtract(0.5,scx);
    // let desiredYposition=math.subtract(scy,0.5);
    // let ccx=math.sqrt(math.divide(math.subtract(scx,hcx),2))
    // let ccy=math.sqrt(math.divide(math.subtract(scx,hcx),2))
    // let desiredXpositionHip=math.subtract(0.5, hcx);
    // let desiredYpositionHip= math.subtract(hcy, 0.5);
    // let renderXval=math.multiply(desiredXposition,scaledZposition, proj0.current,2)
    // let renderZval=math.add(math.multiply(proj10.current,dTempS),proj14.current)
    // let renderYval=math.multiply(desiredYposition,scaledZposition, proj5.current,2)
    //  renderX.current=renderXval // math.multiply(desiredXpositionHip,scaledZposition,proj5.current);
    //  renderY.current=renderYval
    //  renderZ.current=renderZval //dTemp;
    // go go go
    let LEQuatModelOffset = math.matrix([[quatLE.toArray()[0][0], -quatLE.toArray()[0][1], quatLE.toArray()[0][2], quatLE.toArray()[0][3]]]);
    //   LEQuatModel.current=LEQuatModelOffset;
    //   LWAngModel.current=angLW;
    let REQuatModelOffset = math.matrix([[quatRE.toArray()[0][0], -quatRE.toArray()[0][1], quatRE.toArray()[0][2], quatRE.toArray()[0][3]]]);
    //   REQuatModel.current=REQuatModelOffset;
    //   RWAngModel.current=angRW;
    let pivotBodyPins = math.zeros(3, 4);
    pivotBodyPins.subset(math.index(0, 0), 0);
    pivotBodyPins.subset(math.index(1, 0), 0);
    pivotBodyPins.subset(math.index(2, 0), 5);
    pivotBodyPins.subset(math.index(0, 1), 0);
    pivotBodyPins.subset(math.index(1, 1), 0);
    pivotBodyPins.subset(math.index(2, 1), -5);
    pivotBodyPins.subset(math.index(0, 2), math.divide(math.multiply(bodyScale, HW), 2));
    pivotBodyPins.subset(math.index(1, 2), math.multiply(-bodyScale, L, 1));
    pivotBodyPins.subset(math.index(2, 2), 0);
    pivotBodyPins.subset(math.index(0, 3), math.divide(math.multiply(bodyScale, HW), -2));
    pivotBodyPins.subset(math.index(1, 3), math.multiply(-bodyScale, L));
    pivotBodyPins.subset(math.index(2, 3), 0);
    let rotTorsoCurrent = LCGD2(bodyQuatOffset).LGC;
    let pivotLEx = math.divide(math.multiply(bodyScale, SW), 2);
    let pivotLEy = 0; //math.divide(renderLscale.current,2);
    let pivotLEz = 0;
    let pivotLEshoulder = math.zeros(3, 1);
    pivotLEshoulder.subset(math.index(0, 0), pivotLEx);
    pivotLEshoulder.subset(math.index(1, 0), pivotLEy);
    pivotLEshoulder.subset(math.index(2, 0), pivotLEz);
    let pivotLE = math.zeros(3, 1);
    pivotLE.subset(math.index(0, 0), 0);
    pivotLE.subset(math.index(1, 0), math.multiply(-bodyScale, LE));
    pivotLE.subset(math.index(2, 0), 0);
    let pivotREx = math.divide(math.multiply(bodyScale, SW), -2);
    let pivotREy = 0; //math.divide(renderLscale.current,2);
    let pivotREz = 0;
    let pivotREshoulder = math.zeros(3, 1);
    pivotREshoulder.subset(math.index(0, 0), pivotREx);
    pivotREshoulder.subset(math.index(1, 0), pivotREy);
    pivotREshoulder.subset(math.index(2, 0), pivotREz);
    let pivotRE = math.zeros(3, 1);
    pivotRE.subset(math.index(0, 0), 0);
    pivotRE.subset(math.index(1, 0), math.multiply(-bodyScale, RE));
    pivotRE.subset(math.index(2, 0), 0);
    let ithetaLx = 0; //math.divide(math.pi,4)
    let offsetLx = math.matrix([[0, math.sin(math.divide(ithetaLx, 2)), 0, math.cos(math.divide(ithetaLx, 2))]]);
    let rotLECurrent = LCGD2(LEQuatModelOffset).LGC;
    let rotLEx = LCGD2(quatMult2(LEQuatModelOffset, offsetLx).quat).LGC;
    let LSposition = math.multiply(rotTorsoCurrent, pivotLEshoulder);
    let LEposition = math.multiply(rotTorsoCurrent, math.add(pivotLEshoulder, math.multiply(rotLEx, pivotLE)));
    // LSposition.current=currentPosiLEshoulder;
    // LEposition.current=currentPosiLEcenter;
    let itheta1L = 0; //math.divide(math.pi,-2)
    let offset1L = math.matrix([[0, 0, math.sin(math.divide(itheta1L, 2)), math.cos(math.divide(itheta1L, 2))]]);
    let LEquatCurrent = quatMult2(LEQuatModelOffset, offset1L).quat;
    let LSrotation = quatMult2(bodyQuatOffset, LEquatCurrent).quat;
    // let LSrotation=LEquatCurrent2
    let sAngLW = math.sin(math.divide(angLW, -2));
    let cAngLW = math.cos(math.divide(angLW, -2));
    let itheta2L = math.divide(math.pi, 2);
    let offset2L = math.matrix([[0, math.sin(math.divide(itheta2L, 2)), 0, math.cos(math.divide(itheta2L, 2))]]);
    let LWQuatModel = math.matrix([[sAngLW, 0, 0, cAngLW]]);
    let LWquatCurrent = quatMult2(LWQuatModel, offset2L).quat;
    let LWquatCurrent2 = quatMult2(LEQuatModelOffset, LWquatCurrent).quat;
    let LErotation = quatMult2(bodyQuatOffset, LWquatCurrent2).quat;
    // LErotation.current= LWquatCurrent3;
    let rotRECurrent = LCGD2(REQuatModelOffset).LGC;
    let RSposition = math.multiply(rotTorsoCurrent, pivotREshoulder);
    let REposition = math.multiply(rotTorsoCurrent, math.add(pivotREshoulder, math.multiply(rotRECurrent, pivotRE)));
    // RSposition.current=currentPosiREshoulder
    // REposition.current=currentPosiRWcenter
    let itheta1R = math.divide(math.pi, 1);
    let offset1R = math.matrix([[0, 0, math.sin(math.divide(itheta1R, 2)), math.cos(math.divide(itheta1R, 2))]]);
    let REquatCurrent = quatMult2(REQuatModelOffset, offset1R).quat;
    let RSrotation = quatMult2(bodyQuatOffset, REquatCurrent).quat;
    // RSrotation.current=REquatCurrent2;
    let itheta2R = 0; //math.divide(math.pi,-2)
    let offset2R = math.matrix([[0, 0, math.sin(math.divide(itheta2R, 2)), math.cos(math.divide(itheta2R, 2))]]);
    let sAngRW = math.sin(math.divide(angRW, -2));
    let cAngRW = math.cos(math.divide(angRW, -2));
    let RWQuatModel = math.matrix([[sAngRW, 0, 0, cAngRW]]);
    let RWquatCurrent = quatMult2(RWQuatModel, offset2R).quat;
    let RWquatCurrent2 = quatMult2(REQuatModelOffset, RWquatCurrent).quat;
    let RErotation = quatMult2(bodyQuatOffset, RWquatCurrent2).quat;
    // RErotation.current=RWquatCurrent3;
    // if (RSrotationOffsetReference.current){
    //  RSrotationOffset.current=quatMultInv(RSrotation.current, RSrotationOffsetReference.current).quat;
    // } else {
    // RSrotationOffset.current=RSrotation.current;
    // }
    // if (LSrotationOffsetReference.current){
    // LSrotationOffset.current=quatMultInv(LSrotation.current, LSrotationOffsetReference.current).quat;
    // } else {
    // LSrotationOffset.current=LSrotation.current;
    // }
    // if (RErotationOffsetReference.current){
    // RErotationOffset.current=quatMultInv(RErotation.current,RErotationOffsetReference.current).quat;
    // } else {
    // RErotationOffset.current=RErotation.current;
    // }
    // if (LErotationOffsetReference.current){
    // LErotationOffset.current=quatMultInv(LErotation.current,LErotationOffsetReference.current).quat;
    // } else {
    // LErotationOffset.current=LErotation.current;
    // }
    // if (BProtationOffsetReference.current){
    // BProtationOffset.current=quatMultInv(BProtation.current,BProtationOffsetReference.current).quat;
    // } else {
    // BProtationOffset.current=BProtation.current;
    // }
    let currentBodyPins = math.multiply(rotTorsoCurrent, pivotBodyPins);
    // BProtation.current=bodyQuatOffset;
    let BPpositionFront = math.subset(currentBodyPins, math.index([0, 1, 2], 0));
    let BPpositionBack = math.subset(currentBodyPins, math.index([0, 1, 2], 1));
    let BPpositionLeft = math.subset(currentBodyPins, math.index([0, 1, 2], 2));
    let BPpositionRight = math.subset(currentBodyPins, math.index([0, 1, 2], 3));
    // // instead for cylinder
    // renderSWscale.current=math.multiply( bodyScale,SW);
    // renderHWscale.current=math.multiply( bodyScale,HW);
    // renderLscale.current=math.multiply( bodyScale,L);
    // renderLEscale.current=math.multiply( bodyScale,LE);
    // renderLWscale.current=math.multiply( bodyScale,LW);
    // renderREscale.current=math.multiply(bodyScale,RE);
    // renderRWscale.current=math.multiply(bodyScale,RW);
    return { dTemp: dTemp, scx: scx, scy: scy, bodyQuat: bodyQuatOffset,
        L: L, HW: HW, SW: SW, LE: LE, RE: RE, LW: LW, RW: RW,
        quatLE: LEQuatModelOffset, quatRE: REQuatModelOffset,
        angLW: angLW, angRW: angRW, alpha: alphaSF,
        LSposition: LSposition, LSrotation: LSrotation, RSposition: RSposition, RSrotation: RSrotation,
        LEposition: LEposition, LErotation: LErotation, REposition: REposition, RErotation: RErotation,
        BProtation: bodyQuatOffset, BPpositionFront: BPpositionFront, BPpositionBack: BPpositionBack,
        BPpositionLeft: BPpositionLeft, BPpositionRight: BPpositionRight };
    // renderLExz.current=math.multiply(bodyExtraScale,bodyScale, LE, LExzRatio);
}
// export const EZClothPiece = forwardRef((props, refs) => {
//     let nBalls = props.nBalls;
//     let pointsPos = props.pointsPos;
//     let indices = props.indices;
//     let nUniqueBallLinks = props.nUniqueBallLinks;
//     let uniqueBallLinks = props.uniqueBallLinks;
//     // let body1mesh = useRef();
//     let refMesh = useRef();
//     let firstCloth = true;
//     // let stitchCounter=0;
//     let velocityScale = 0.7;
//     let maxFactor = 300;
//     // const raycaster = new THREE.Raycaster()
//     useFrame(() => {
//         // if (props.recordMode==true){
//         // refs.current.forEach((ref, vi) => {
//         //   if (ref.current){
//         //   const p = ref.current.translation();
//         //   let index_vi=3*vi
//         //   if (props.frontBack>0){
//         //     pointsPosRef1.current[index_vi]=p.x
//         //     pointsPosRef1.current[index_vi+1]=p.y
//         //     pointsPosRef1.current[index_vi+2]=p.z
//         //     } else {
//         //       pointsPosRef2.current[index_vi]=p.x
//         //       pointsPosRef2.current[index_vi+1]=p.y
//         //       pointsPosRef2.current[index_vi+2]=p.z
//         //     }
//         //   }
//         //   })
//         // }
//         if (firstCloth == true) {
//             //     // firstCloth boolean used to set bufferGeometry 
//             let points = new BufferAttribute(new Float32Array(pointsPos), 3);
//             refMesh.current.geometry.setAttribute('position', points);
//             refMesh.current.geometry.setIndex(indices);
//             firstCloth = false;
//         }
//         else {
//             const pos = refMesh.current.geometry.getAttribute('position');
//             refs.current.forEach((ref, vi) => {
//                 const p = ref.current.translation();
//                 let index_vi = 3 * vi;
//                 pos.array[index_vi] = p.x;
//                 pos.array[index_vi + 1] = p.y;
//                 pos.array[index_vi + 2] = p.z;
//             });
//             refMesh.current.geometry.setAttribute('position', pos);
//             refMesh.current.geometry.attributes.position.needsUpdate = true;
//             refMesh.current.geometry.computeVertexNormals();
//             refMesh.current.geometry.normalizeNormals();
//         }
//         // update anchors positions
//         // remember to turn on anchorRef generation
//     });
//     return (<Suspense fallback={null}>    

//         {/* <Cylinder position={[0,math.divide(renderLscale.current,-2), ppDz]}
//                         args={[math.divide(renderSWscale.current,2), math.divide(renderHWscale.current,2), renderLscale.current, cylinderRadialSegments, cylinderHeightSegments]}
//                         visible={false} ref={body1mesh}/>  */}

//       <group>
//       <mesh ref={refMesh}>
//       <bufferGeometry />
//       <meshStandardMaterial color="red" side={DoubleSide}/>
//       {/* <meshBasicMaterial color="hotpink" wireframe={true}/> */}
//        {/* <meshPhongMaterial color="#ff0000" opacity={0.1} transparent /> */}
//       </mesh> 
      

//       {Array.from({ length: nBalls }).map((_, i) => (<RigidBody collisionGroups={interactionGroups(1, 0)} position={[pointsPos[3 * i], pointsPos[3 * i + 1], pointsPos[3 * i + 2]]} includeInvisible 
//         // velocity={[0, 0, 0]}
//         // rotation={[0, 0, 0, 1]}
//         // linearDamping={clothDamping} 
//         // angularDamping={clothAngularDamping}
//         friction={props.clothFriction} 
//         // density={clothDensity}
//         mass={1000} 
//         // enabledRotations={[false]}
//         gravityScale={2} restitution={props.clothRestition} colliders="ball" ref={refs.current[i]} key={i + props.frontBack + "ball_pos"} ccd="true">
//         <Sphere args={[props.clothBallSize]} visible={false}/>

//         </RigidBody>))}


//   {/* below renders all ball cloth joints
//               TODO I need a for loop here instead of these duplicated lines below to make it cleaner */}
//           {refs.current.map((_, i) => (nUniqueBallLinks[i] > 0) && (<RopeJointXYZ0 b={refs.current[i]} a={refs.current[uniqueBallLinks[i][0]]} dXin={-pointsPos[3 * i] + pointsPos[3 * uniqueBallLinks[i][0]]} dYin={-pointsPos[3 * i + 1] + pointsPos[3 * uniqueBallLinks[i][0] + 1]} dZin={-pointsPos[3 * i + 2] + pointsPos[3 * uniqueBallLinks[i][0] + 2]} key={i + props.frontBack + 'nuxyj1'}/>))}  
//    {refs.current.map((_, i) => (nUniqueBallLinks[i] > 1) && (<RopeJointXYZ0 b={refs.current[i]} a={refs.current[uniqueBallLinks[i][1]]} dXin={-pointsPos[3 * i] + pointsPos[3 * uniqueBallLinks[i][1]]} dYin={-pointsPos[3 * i + 1] + pointsPos[3 * uniqueBallLinks[i][1] + 1]} dZin={-pointsPos[3 * i + 2] + pointsPos[3 * uniqueBallLinks[i][1] + 2]} key={i + props.frontBack + 'nuxyj2'}/>))}
//  {refs.current.map((_, i) => (nUniqueBallLinks[i] > 2) && (<RopeJointXYZ0 b={refs.current[i]} a={refs.current[uniqueBallLinks[i][2]]} dXin={-pointsPos[3 * i] + pointsPos[3 * uniqueBallLinks[i][2]]} dYin={-pointsPos[3 * i + 1] + pointsPos[3 * uniqueBallLinks[i][2] + 1]} dZin={-pointsPos[3 * i + 2] + pointsPos[3 * uniqueBallLinks[i][2] + 2]} key={i + props.frontBack + 'nuxyj3'}/>))}
//    {refs.current.map((_, i) => (nUniqueBallLinks[i] > 3) && (<RopeJointXYZ0 b={refs.current[i]} a={refs.current[uniqueBallLinks[i][3]]} dXin={-pointsPos[3 * i] + pointsPos[3 * uniqueBallLinks[i][3]]} dYin={-pointsPos[3 * i + 1] + pointsPos[3 * uniqueBallLinks[i][3] + 1]} dZin={-pointsPos[3 * i + 2] + pointsPos[3 * uniqueBallLinks[i][3] + 2]} key={i + props.frontBack + 'uxyj1'}/>))}
//    {refs.current.map((_, i) => (nUniqueBallLinks[i] > 4) && (<RopeJointXYZ0 b={refs.current[i]} a={refs.current[uniqueBallLinks[i][4]]} dXin={-pointsPos[3 * i] + pointsPos[3 * uniqueBallLinks[i][4]]} dYin={-pointsPos[3 * i + 1] + pointsPos[3 * uniqueBallLinks[i][4] + 1]} dZin={-pointsPos[3 * i + 2] + pointsPos[3 * uniqueBallLinks[i][4] + 2]} key={i + props.frontBack + 'uxyj2'}/>))}
//   {refs.current.map((_, i) => (nUniqueBallLinks[i] > 5) && (<RopeJointXYZ0 b={refs.current[i]} a={refs.current[uniqueBallLinks[i][5]]} dXin={-pointsPos[3 * i] + pointsPos[3 * uniqueBallLinks[i][5]]} dYin={-pointsPos[3 * i + 1] + pointsPos[3 * uniqueBallLinks[i][5] + 1]} dZin={-pointsPos[3 * i + 2] + pointsPos[3 * uniqueBallLinks[i][5] + 2]} key={i + props.frontBack + 'uxyj3'}/>))}         
//       </group>
//     </Suspense>);
// });
// function createPinStitch( nPins, pointPosPins, nBalls, pointsPosBalls){
export function createPinStitchM(nPins, pointPosPins, nBalls, pointsPosBalls, minBallDistance) {
    // this method will only produce last min disrance pin anchor to cloth joiint
    // TODO may want to allow more than one joint
    // let nPins=1;
    let nAnchors = 0;
    let anchorPosVec = [];
    let anchorIndexVecs = [];
    let pinIndexVecs = [];
    let pinConnectionLengthX = [];
    let pinConnectionLengthY = [];
    let pinConnectionLengthZ = [];
    // let posX=pointsPosBalls[0];
    // let posY=pointsPosBalls[1];
    // let posZ=pointsPosBalls[2];
    // let pinX=pointPosPins[0]
    // let pinY=pointPosPins[1]
    // let pinZ=pointPosPins[2]
    let anchorIndexVec = [];
    let scaleFactor = 1;
    // for j pins -- 1 connection each append pinList
    //
    for (var j = 0; j < (nPins); j++) {
        let pinX = pointPosPins[3 * j];
        let pinY = pointPosPins[3 * j + 1];
        let pinZ = pointPosPins[3 * j + 2];
        for (var i = 0; i < (nBalls); i++) {
            let posX = pointsPosBalls[3 * i];
            let posY = pointsPosBalls[3 * i + 1];
            let posZ = pointsPosBalls[3 * i + 2];
            let testDistance = math.sqrt(math.add(math.pow(math.subtract(pinX, posX), 2), math.pow(math.subtract(pinY, posY), 2), math.pow(math.subtract(pinZ, posZ), 2)));
            // let testDistance0=math.divide(math.abs(math.subtract(term1,term2)),denominator)
            if (testDistance < minBallDistance) {
                // minj=i;
                // minBallDistance=testDistance;
                pinConnectionLengthX.push(math.multiply(scaleFactor, math.subtract(pinX, pointsPosBalls[3 * i])));
                pinConnectionLengthY.push(math.multiply(scaleFactor, math.subtract(pinY, pointsPosBalls[3 * i + 1])));
                pinConnectionLengthZ.push(math.multiply(scaleFactor, math.subtract(pinZ, pointsPosBalls[3 * i + 2])));
                anchorPosVec.push(pointsPosBalls[3 * i]);
                anchorPosVec.push(pointsPosBalls[3 * i + 1]);
                anchorPosVec.push(pointsPosBalls[3 * i + 2]);
                anchorIndexVec.push(i);
                nAnchors = nAnchors + 1;
            }
        }
    }
    //pinConnectionIndex nPinConnections pinIndex1 pinConnectionLength1
    return { nPinConnections: nAnchors, pinIndexVec: 0, pinConnectionIndex: anchorIndexVec, pinConnectionLengthX: pinConnectionLengthX, pinConnectionLengthY: pinConnectionLengthY, pinConnectionLengthZ: pinConnectionLengthZ };
}
function pDistance(x, y, x1, y1, x2, y2) {
    var A = x - x1;
    var B = y - y1;
    var C = x2 - x1;
    var D = y2 - y1;
    var dot = A * C + B * D;
    var len_sq = C * C + D * D;
    var param = -1;
    if (len_sq != 0) //in case of 0 length line
        param = dot / len_sq;
    var xx, yy;
    if (param < 0) {
        xx = x1;
        yy = y1;
    }
    else if (param > 1) {
        xx = x2;
        yy = y2;
    }
    else {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }
    var dx = x - xx;
    var dy = y - yy;
    return Math.sqrt(dx * dx + dy * dy);
}
// ********************************  
// export const Slab = forwardRef((props, ref1) => {
//     // let slabDepth=10;
//     let slabZposition = math.subtract(props.ppDz, math.divide(props.slabDepth, 2));
//     // useFrame(() => {
//     //       if (ref1.current) {
//     //         ref1.current.setNextKinematicRotation({
//     //           x: 0,
//     //           y: 0,
//     //           z: 0,
//     //           w: 1
//     //         });
//     //         ref1.current.setNextKinematicTranslation({
//     //           x: 0,
//     //           y: 0,
//     //           z: ppDz
//     //          }); 
//     //           }
//     //         })
//     return (<group position={[0, 0, slabZposition]}> 
//               <RigidBody ref={ref1} type="kinematicPosition" gravityScale={0} restitution={0} collisionGroups={interactionGroups(0, 1)}>
//            {/* slab depth is depth to prevent balls colliding*/}
//                <CuboidCollider args={[100, 100, props.slabDepth]}/>
//             </RigidBody>
//               </group>);
// });
// export const KineticPinRS = forwardRef((props, ref) => {
//     // LSposition.current
//     // LSrotation.current
//     useBeforePhysicsStep(() => {
//         if (ref.current) {
//             ref.current.setNextKinematicTranslation({
//                 x: math.add(props.ppDxC, props.position.current.toArray()[0][0]),
//                 y: math.add(0, props.position.current.toArray()[1][0]),
//                 z: math.add(props.ppDz, props.position.current.toArray()[2][0])
//             });
//             ref.current.setNextKinematicRotation({
//                 x: props.rotation.current.toArray()[0][0],
//                 y: props.rotation.current.toArray()[0][1],
//                 z: props.rotation.current.toArray()[0][2],
//                 w: props.rotation.current.toArray()[0][3]
//             });
//         }
//     });
//     return (<Suspense fallback={null}>    
//                 <RigidBody collisionGroups={interactionGroups(0, 2)} includeInvisible 
//     //  ??????????????? ppDy
//     position={[math.add(props.ppDxC, props.position.current.toArray()[0][0]), math.add(0, props.position.current.toArray()[1][0]), math.add(props.ppDz, props.position.current.toArray()[2][0])]} 
//     // gravityScale={[0]}
//     // restitution={[0]} 
//     colliders={false} ref={ref} type="kinematicPosition" key={props.akey}>
                  
//                   <Sphere args={[2]} visible={false}/>
//                 </RigidBody>
            
      
//           </Suspense>);
// });
export function createXstitch(startX, stopX, startY, stopY, nBalls, pointsPosIn) {
    // startX=ppDxC-(TW/2 + sleeve2)
    // stopX=ppDxC-collar1/2
    // let anchorStepSize=(stopX-startX)/nAnchors;
    // debugger;
    let anchorPosVec = [];
    let anchorIndexVec = [];
    let X1 = startX;
    let X2 = stopX;
    let Y1 = startY;
    let Y2 = stopY;
    let minBallDistance = 1;
    let minj = 0;
    let nAnchors = 0;
    for (var i = 0; i < (nBalls); i++) {
        let posX = pointsPosIn[3 * i];
        let posY = pointsPosIn[3 * i + 1];
        let posZ = pointsPosIn[3 * i + 2];
        // anchorPos[3*i]=positionX;
        // anchorPos[3*i+1]=positionY;
        // anchorPos[3*i+2]=positionZ;
        // (X2-X1)(Y1-Yo) - (X1-Xo)(Y2-Y1)
        // let term1=math.multiply(math.subtract(X2,X1),math.subtract(Y1,posY))
        // let term2=math.multiply(math.subtract(X1,posX),math.subtract(Y2,Y1))
        // let denominator=math.sqrt(math.add(math.pow(math.subtract(X2,X1),2),math.pow(math.subtract(Y2,Y1),2)))
        let testDistance = pDistance(posX, posY, X1, Y1, X2, Y2);
        // let testDistance0=math.divide(math.abs(math.subtract(term1,term2)),denominator)
        if (testDistance < minBallDistance) {
            // minj=i;
            // minBallDistance=testDistance;
            anchorPosVec.push(pointsPosIn[3 * i]);
            anchorPosVec.push(pointsPosIn[3 * i + 1]);
            anchorPosVec.push(pointsPosIn[3 * i + 2]);
            anchorIndexVec.push(i);
            nAnchors = nAnchors + 1;
        }
    }
    return { indexVec: anchorIndexVec, posVec: anchorPosVec, nAnchors: nAnchors };
}
// export const RopeJointXYZ0 = ({ a, b, dXin, dYin, dZin }) => {
//     const joint = useSphericalJoint(a, b, [
//         [-dXin, -dYin, -dZin],
//         [0, 0, 0],
//     ]);
//     return null;
// };
export function createCloth(ballSpaceX, ballSpaceY, ppDxI, ppDyI, ppDzI, clothScale, TW, TH, sleeve1, sleeve2, collar1, collar2, maxTriangleSize) {
    // let TWaspect= 0.7;
    // let sleeve1Aspect=0.6; //height of sleeve
    // let sleeve2Aspect=0.5; // length of sleeve
    // let sleeve3Aspect=1-sleeve1Aspect;
    // let collar1Aspect=0.3; // width collar
    // let collar2Aspect=0.5;
    //input: TW TH sleeve1 sleeve2 collar1 collar2
    // cloth is made by drawing outline
    const data0 = {
        type: "FeatureCollection",
        features: [{
                "type": "Feature",
                "properties": {
                    "category": 0,
                },
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[
                            [ppDxI - TW / 2, ppDyI - TH],
                            [ppDxI - TW / 2, ppDyI - TH + sleeve1],
                            [ppDxI - (TW / 2 + sleeve2), ppDyI - TH + sleeve1],
                            [ppDxI - (TW / 2 + sleeve2), ppDyI],
                            [ppDxI - collar1 / 2, ppDyI],
                            [ppDxI - collar1 / 2, ppDyI - collar2],
                            [ppDxI + collar1 / 2, ppDyI - collar2],
                            [ppDxI + collar1 / 2, ppDyI],
                            [ppDxI + TW / 2 + sleeve2, ppDyI],
                            [ppDxI + TW / 2 + sleeve2, ppDyI - TH + sleeve1],
                            [ppDxI + TW / 2, ppDyI - TH + sleeve1],
                            [ppDxI + TW / 2, ppDyI - TH],
                        ]]
                }
            }]
    };
    // create grid points 
    let gridPointsOutput = createGridPoints(data0, ballSpaceX, ballSpaceY, ppDzI); //get points array
    let points3d = gridPointsOutput.points3d;
    let histogramX = gridPointsOutput.histogramX;
    let indexValX = gridPointsOutput.indexX;
    let indexValY = gridPointsOutput.indexY;
    let clothLinks = gridPointsOutput.links;
    let stitchIndexX = [];
    for (var i = 0; i < (histogramX.length - 2); i++) {
        let diffVal = math.abs(histogramX[i + 1] - histogramX[i]);
        if (diffVal > 0) {
            stitchIndexX.push(indexValX[i]);
            stitchIndexX.push(indexValX[i + 1]);
        }
    }
    let histogramY = gridPointsOutput.histogramY;
    let stitchIndexY = [];
    for (var i = 2; i < (histogramY.length - 2); i++) {
        let diffVal = math.abs(histogramY[i + 1] - histogramY[i]);
        if (diffVal > 0) {
            stitchIndexY.push(indexValY[i]);
            stitchIndexY.push(indexValY[i + 1]);
        }
    }
    // for stitching location attribution
    // --x will have 2 changes identifying sleeves
    // y should have 1 below sleeves, then sleeve bottom
    // achieved stitching
    let output = createGeometry(points3d, maxTriangleSize);
    let indices = output.indices;
    let newPoints = output.points;
    const nBalls = newPoints.length;
    let firstCloth = true;
    let pointsPos = new Float32Array(3 * nBalls);
    // need additional joints- for pointPos index is 1 2 3
    // need for x for loop
    let minX = 1000;
    let minY = 1000;
    let maxX = -1000;
    let maxY = -1000;
    for (var i = 0; i < (nBalls); i++) {
        let positionX = newPoints[i].x;
        let positionY = newPoints[i].y;
        let positionZ = newPoints[i].z;
        pointsPos[3 * i] = positionX;
        pointsPos[3 * i + 1] = positionY;
        pointsPos[3 * i + 2] = positionZ;
        if (positionX < minX) {
            minX = positionX;
        }
        if (positionX > maxX) {
            maxX = positionX;
        }
        if (positionY < minY) {
            minY = positionY;
        }
        if (positionY > maxY) {
            maxY = positionY;
        }
    }
    let nIndices = indices.length;
    var allLinks = new Array(nIndices + clothLinks.length);
    // var allLinks=new Array(nIndices);
    const nUniqueBallLinks = new Float32Array(nBalls);
    var allLinksSorted = new Array(nIndices);
    // follow trimesh traingle index pattern ABC BCA & CAB
    Array.from({ length: nIndices + clothLinks.length }).map((_, i) => (
    //  Array.from({ length: nIndices }).map((_, i) =>(
    allLinks[i] = new Array()));
    Array.from({ length: nIndices / 3 }).map((_, i) => (allLinks[3 * i].push([indices[3 * i], indices[3 * i + 1]])));
    Array.from({ length: nIndices / 3 }).map((_, i) => (allLinks[3 * i + 1].push([indices[3 * i + 1], indices[3 * i + 2]])));
    Array.from({ length: nIndices / 3 }).map((_, i) => (allLinks[3 * i + 2].push([indices[3 * i + 2], indices[3 * i]])));
    Array.from({ length: clothLinks.length }).map((_, i) => (allLinks[nIndices + i].push([clothLinks[i][0], clothLinks[i][1]])));
    Array.from({ length: allLinks.length }).map((_, i) => (allLinksSorted[i] = (allLinks[i][0][0] > allLinks[i][0][1]) ? [allLinks[i][0][0], allLinks[i][0][1]] : [allLinks[i][0][1], allLinks[i][0][0]]));
    // need to sort links, remove reduncies as each link becomes joint
    let uniqueLinks = multiDimensionalUnique(allLinksSorted);
    var uniqueBallLinks = new Array(nBalls);
    Array.from({ length: nBalls }).map((_, i) => (uniqueBallLinks[i] = new Array()));
    Array.from({ length: uniqueLinks.length }).map((_, i) => (uniqueBallLinks[uniqueLinks[i][0]].push(uniqueLinks[i][1])));
    Array.from({ length: uniqueLinks.length }).map((_, i) => (nUniqueBallLinks[uniqueLinks[i][0]] = nUniqueBallLinks[uniqueLinks[i][0]] + 1));
    const lengthLinksX = new Float32Array(uniqueLinks.length);
    const lengthLinksY = new Float32Array(uniqueLinks.length);
    const lengthLinksZ = new Float32Array(uniqueLinks.length);
    const lengthLinks = new Float32Array(uniqueLinks.length);
    Array.from({ length: uniqueLinks.length }).map((_, i) => (lengthLinksX[i] = math.subtract(pointsPos[3 * uniqueLinks[i][1]], pointsPos[3 * uniqueLinks[i][0]])));
    Array.from({ length: uniqueLinks.length }).map((_, i) => (lengthLinksY[i] = math.subtract(pointsPos[3 * uniqueLinks[i][1]] + 1, pointsPos[3 * uniqueLinks[i][0] + 1])));
    Array.from({ length: uniqueLinks.length }).map((_, i) => (lengthLinksZ[i] = math.subtract(pointsPos[3 * uniqueLinks[i][1]] + 2, pointsPos[3 * uniqueLinks[i][0] + 2])));
    Array.from({ length: uniqueLinks.length }).map((_, i) => (lengthLinks[i] = math.norm([lengthLinksX[i], lengthLinksY[i], lengthLinksZ[i]])));
    // output return nBalls pointsPos uniqueBallLinks
    return { nBallsOut: nBalls, pointsPosOut: pointsPos, uniqueBallLinksOut: uniqueBallLinks,
        indicesOut: indices, nUniqueBallLinksOut: nUniqueBallLinks, minX: minX, maxX: maxX, minY: minY, maxY: maxY,
        stitchIndexX: stitchIndexX, stitchIndexY: stitchIndexY, nLinks: uniqueLinks.length, lengthLinks: lengthLinks };
}
