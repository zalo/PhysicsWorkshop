import * as THREE from '../node_modules/three/build/three.module.js';
import {
    MeshBVH, computeBoundsTree, disposeBoundsTree,
    computeBatchedBoundsTree, disposeBatchedBoundsTree, acceleratedRaycast,
} from '../node_modules/three-mesh-bvh/src/index.js';

// Add the extension functions
THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
THREE.Mesh.prototype.raycast = acceleratedRaycast;
THREE.BatchedMesh.prototype.computeBoundsTree = computeBatchedBoundsTree;
THREE.BatchedMesh.prototype.disposeBoundsTree = disposeBatchedBoundsTree;
THREE.BatchedMesh.prototype.raycast = acceleratedRaycast;

/** A simple verlet + three-mesh-bvh Physics Engine for handling non-convex meshes */
export default class PhysicsManager {
    constructor(scene) {
        this.initialized = false;
        /** @type {THREE.Mesh[]} */
        this.physicsObjects = [];
        this.curOffset      = new THREE.Vector3   (0, 0, 0);
        this.curQuaternion  = new THREE.Quaternion(0, 0, 0, 1);
        this.tempVec        = new THREE.Vector3   (0, 0, 0);
        this.tempVec2       = new THREE.Vector3   (0, 0, 0);
        this.forward        = new THREE.Vector3   (0, 0, 1);
        this.tempNormal     = new THREE.Vector3   (0, 0, 0);
        this.tempAvg        = new THREE.Vector3   (0, 0, 0);
        this.tempRay        = new THREE.Ray       (new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 1));
        this.tempMatrix     = new THREE.Matrix4  ();

        this.raycaster = new THREE.Raycaster();
        this.raycaster.firstHitOnly = true;
        //this.raycaster.intersectObjects( [  ] );

        this.debugSpheres   = new THREE.InstancedBufferGeometry().copy(new THREE.SphereGeometry(1.0, 8, 8));
        this.debugMaterial  = new THREE.MeshBasicMaterial({ color: 0xaa0000 });
        this.debugMesh      = new THREE.InstancedMesh(this.debugSpheres, this.debugMaterial, 10240);
        this.debugMesh.visible = true;
        scene.add(this.debugMesh);

        this.setup();
    }
    async setup() { }

    /** Turn a mesh into a physics object by adding fields 
     * @param {THREE.Mesh} mesh - The mesh to turn into a physics object */
    createPhysicsObject(mesh) {
        mesh.updateMatrixWorld(); // Ensure the world matrix is up to date

        // Construct a bounding volume hierarchy for the mesh
        /** @type {THREE.Box3} */
        mesh.userData.boundingBox = new THREE.Box3().setFromObject(mesh);
        if (!mesh.geometry.boundsTree) {
            mesh.geometry.computeBoundsTree();
        }

        let boxHelper = new THREE.Box3Helper(mesh.userData.boundingBox, 0xffffff);
        mesh.parent.parent.add(boxHelper);
        boxHelper.material.depthTest = false;
        boxHelper.material.transparent = true;
        mesh.userData.boxHelper = boxHelper;
        mesh.userData.boxHelper.updateMatrixWorld();
        console.log(mesh.parent, mesh.userData.boxHelper);

        let physicsReferencePositions = this.packSpheres(mesh.geometry, 100);
        let numCollisionProxyPoints = Math.max(Math.floor(mesh.geometry.userData.surfaceArea * 200), 30);
        console.log("Volume:", mesh.geometry.userData.surfaceArea, "Num Points:", numCollisionProxyPoints);
        physicsReferencePositions = this.packSpheres(mesh.geometry, numCollisionProxyPoints);

        // Create a physics object from the mesh
        mesh.userData.isPhysicsObject = true;
        mesh.userData.physics = {
            needsUpdate       : false,
            referencePositions: physicsReferencePositions,
            positions         : new Float32Array(physicsReferencePositions),
            previousPositions : new Float32Array(physicsReferencePositions)
        };

        console.log("MESH CREATED! Vertices:", physicsReferencePositions.length/4);

        // Transform the positions to world space
        for(let i = 0; i < physicsReferencePositions.length; i+=4) {
            mesh.localToWorld(this.tempVec.fromArray(physicsReferencePositions, i));
            mesh.userData.physics.positions        [i]   = this.tempVec.x;
            mesh.userData.physics.positions        [i+1] = this.tempVec.y;
            mesh.userData.physics.positions        [i+2] = this.tempVec.z;
            mesh.userData.physics.previousPositions[i]   = this.tempVec.x;// + Math.random() * 0.0001;
            mesh.userData.physics.previousPositions[i+1] = this.tempVec.y;// + Math.random() * 0.0001;
            mesh.userData.physics.previousPositions[i+2] = this.tempVec.z;// + Math.random() * 0.0001;

            let scale = mesh.userData.physics.positions[i+3];
            this.debugMesh.setMatrixAt(i/4, this.tempMatrix.compose(this.tempVec, this.curQuaternion, this.tempVec2.set(scale, scale, scale)));
        }

        mesh.userData.physics.needsUpdate = true;

        this.physicsObjects.push(mesh);
    }

    /** Update all the physics objects by integrating the positions and updating the meshes  */
    update() {
        let debugSpheres = 0;
        for (let object of this.physicsObjects) {
            if (object.userData.isPhysicsObject) {
                if(object.userData.physics.needsUpdate) {
                    object.userData.physics.needsUpdate = false;

                    if(!object.geometry){ object.userData.isPhysicsObject = false; continue; }

                    // Construct a bounding volume hierarchy for the mesh
                    //object.geometry.computeBoundingBox();
                    /** @type {THREE.Box3} */
                    object.userData.boundingBox.setFromObject(object);
                    object.userData.boxHelper.updateMatrixWorld();
                    if (!object.geometry.boundsTree) {
                        object.geometry.computeBoundsTree();
                    }

                    let numCollisionProxyPoints = Math.max(Math.floor(object.geometry.userData.surfaceArea * 200), 30);
                    console.log("Volume:", object.geometry.userData.surfaceArea, "Num Points:", numCollisionProxyPoints);
                    object.userData.physics.referencePositions = this.packSpheres(object.geometry, numCollisionProxyPoints);
                    object.userData.physics.positions         = new Float32Array(object.userData.physics.referencePositions);
                    object.userData.physics.previousPositions = new Float32Array(object.userData.physics.referencePositions);

                    // Transform the positions to world space
                    // TODO: Figure out a way to assign these so that velocities are maintained cheaply!
                    object.updateMatrixWorld(); // Ensure the world matrix is up to date
                    for(let i = 0; i < object.userData.physics.positions.length; i+=4) {
                        object.localToWorld(this.tempVec.fromArray(object.userData.physics.referencePositions, i));
                        object.userData.physics.positions        [i]   = this.tempVec.x;
                        object.userData.physics.positions        [i+1] = this.tempVec.y;
                        object.userData.physics.positions        [i+2] = this.tempVec.z;
                        object.userData.physics.previousPositions[i]   = this.tempVec.x;
                        object.userData.physics.previousPositions[i+1] = this.tempVec.y;
                        object.userData.physics.previousPositions[i+2] = this.tempVec.z;
            
                        let scale = object.userData.physics.positions[i+3];
                        this.debugMesh.setMatrixAt(debugSpheres, this.tempMatrix.compose(this.tempVec, this.curQuaternion, this.tempVec2.set(scale, scale, scale)));
                        debugSpheres = debugSpheres + 1;
                    }
                }

                // Integrate the positions forward in time
                this.verletIntegrate(object.userData.physics.positions, object.userData.physics.previousPositions);

                // Add Gravity and Collide the positions against the ground
                let collided = [];
                for (let i = 0; i < object.userData.physics.positions.length; i+=4) {
                    // Gravity
                    object.userData.physics.positions[i+1] -= 0.001;

                    // Collide with the ground
                    if (object.userData.physics.positions[i+1] < 0.01){//object.userData.physics.positions[i+3]) {
                        collided.push(i);
                        object.userData.physics.positions[i+1] = 0.01;//object.userData.physics.positions[i+3];
                        // Friction - Mostly just horizontal damping while on the ground
                        let perpendicularVelocityX = object.userData.physics.positions[i]   - object.userData.physics.previousPositions[i];
                        let perpendicularVelocityZ = object.userData.physics.positions[i+2] - object.userData.physics.previousPositions[i+2];
                        object.userData.physics.positions[i]   -= perpendicularVelocityX * 0.25;
                        object.userData.physics.positions[i+2] -= perpendicularVelocityZ * 0.25;
                    }

                }

                // Collide with each other
                object.userData.boxHelper.material.color.setHex(0x00ff00);
                /** @type {THREE.Box3} */
                let ourBounds = object.userData.boundingBox;
                for (let other of this.physicsObjects) {
                    if (other.geometry && other.userData.isPhysicsObject && object !== other) {
                        // Do "broadphase" box-box check
                        /** @type {THREE.Box3} */
                        let bounds = other.userData.boundingBox;
                        if(!bounds) { continue; }//other.userData.boundingBox = new THREE.Box3().setFromObject(other); bounds = other.userData.boundingBox; }

                        if(ourBounds.intersectsBox(bounds)){
                            /** @type {MeshBVH} */
                            let otherBVH = other.geometry.boundsTree;
                            if(!otherBVH) { other.geometry.computeBoundsTree(); otherBVH = other.geometry.boundsTree; }

                            object.userData.boxHelper.material.color.setHex(0xffff00);

                            for (let i = 0; i < object.userData.physics.positions.length; i+=4) {
                                this.tempVec2.set(object.userData.physics.previousPositions[i  ],
                                                  object.userData.physics.previousPositions[i+1],
                                                  object.userData.physics.previousPositions[i+2]);
                                this.tempVec.set(object.userData.physics.positions[i  ],
                                                 object.userData.physics.positions[i+1],
                                                 object.userData.physics.positions[i+2]);

                                // Step 1: Check if the particle is inside another body's bounding box
                                if (bounds.containsPoint(this.tempVec)) {
                                    object.userData.physics.positions[i+3] = 0.02;

                                    // Step 2: Check if the particle is inside the mesh
                                    other.worldToLocal(this.tempVec2.fromArray(object.userData.physics.positions, i));

                                    let hit = otherBVH.raycastFirst( this.tempRay.set(this.tempVec2, this.forward), THREE.DoubleSide );
                                    let isInside = hit && hit.face.normal.dot( this.tempRay.direction ) > 0.0;
                                    if(isInside){
                                        let closestPt = other.localToWorld(otherBVH.closestPointToPoint(this.tempVec2).point);
                                        object.userData.physics.positions[i  ] = closestPt.x;
                                        object.userData.physics.positions[i+1] = closestPt.y;
                                        object.userData.physics.positions[i+2] = closestPt.z;
                                        object.userData.physics.positions[i+3] = 0.1;
                                    }

                                    //// Step 2: Check if the particle has entered the mesh via raycast
                                    //other.worldToLocal(this.tempVec .fromArray(object.userData.physics.positions, i));
                                    //other.worldToLocal(this.tempVec2.fromArray(object.userData.physics.previousPositions, i));

                                    //let dir = this.tempVec.sub(this.tempVec2);
                                    //let distance = dir.length();

                                    //let hit = otherBVH.raycastFirst( this.tempRay.set(this.tempVec2, dir.normalize()), THREE.FrontSide, 0.0, distance );

                                    //if(hit){
                                    //    collided.push(i);
                                    //    let closestPt = other.localToWorld(hit.point);
                                    //    object.userData.physics.positions[i  ] = (closestPt.x*0.9) + (object.userData.physics.previousPositions[i  ]*0.1);
                                    //    object.userData.physics.positions[i+1] = (closestPt.y*0.9) + (object.userData.physics.previousPositions[i+1]*0.1);
                                    //    object.userData.physics.positions[i+2] = (closestPt.z*0.9) + (object.userData.physics.previousPositions[i+2]*0.1);
                                    //    object.userData.physics.positions[i+3] = 0.1;
                                    //}
                                }else{
                                    object.userData.physics.positions[i+3] = 0.01;
                                }
                            }
                        }
                    }
                }

                // Kabsch the mesh vertices to the current positions
                this.kabschPoints(
                    object.userData.physics.referencePositions, 
                    object.userData.physics.positions,
                    object.position, object.quaternion, collided);

                object.updateMatrixWorld(); // Ensure the world matrix is up to date

                /** @type {THREE.Box3} */
                object.userData.boundingBox.setFromObject(object);

                // Transform the positions to world space
                for(let i = 0; i < object.userData.physics.positions.length; i+=4) {
                    //if(collided.includes(i)){ continue; }
                    object.localToWorld(this.tempVec.fromArray(object.userData.physics.referencePositions, i));
                    object.userData.physics.positions[i]   = this.tempVec.x;
                    object.userData.physics.positions[i+1] = this.tempVec.y;
                    object.userData.physics.positions[i+2] = this.tempVec.z;

                    let scale = object.userData.physics.positions[i+3];
                    this.debugMesh.setMatrixAt(debugSpheres, this.tempMatrix.compose(this.tempVec, this.curQuaternion, this.tempVec2.set(scale, scale, scale)));
                    debugSpheres = debugSpheres + 1;
                }
                this.debugMesh.instanceMatrix.needsUpdate = true;
            } else {
                //console.error("How did this get here?  Create physics objects with PhysicsManager.createPhysicsObject()!", object);
                // Skip for empty bodies...
            }
        }
        this.debugMesh.count = debugSpheres;
        this.debugMesh.instanceMatrix.needsUpdate = true;
    }


    /** Criminally simple verlet integration
     * XPBD is a more advanced version that handles elasticity in a more principles fashion */
    verletIntegrate(curPoints, pastPoints) {
        for (let i = 0; i < curPoints.length; i++) {
            if(i % 4 === 3) { continue; } // Skip the radius
            let temp       = curPoints[i];
            curPoints [i] += curPoints[i] - pastPoints[i];
            pastPoints[i]  = temp;
        }
    }

    /** Get the centroid of a set of points */
    getAverage(points, collided = []) {
        let total = 0.0;
        let average = [0, 0, 0];
        for (let i = 0; i < points.length; i += 4) {
            let weight = 1.0;//(collided.includes(i) ? 10.0 : 1.0);
            average[0] += points[i    ] * weight;
            average[1] += points[i + 1] * weight;
            average[2] += points[i + 2] * weight;
            total += weight;
        }
        average[0] /= total;
        average[1] /= total;
        average[2] /= total;
        return average;
    }

    /** Pack a buffer geometry with quasirandom points
     * @param {THREE.BufferGeometry} geometry 
     * @param {number} numSpheres 
     * @returns {Float32Array} */
    packSpheres(geometry, numSpheres) {
        let startTime = performance.now();
        let positions = new Float32Array(numSpheres * 4);

        /** @type {MeshBVH} */
        let bvh = geometry.boundsTree;
        /** @type {THREE.Box3} */
        let bounds = geometry.boundingBox;
        let min    = bounds.min.clone();
        let max    = bounds.max.clone();

        // Expand the bounds slightly
        min.set(min.x-0.2, min.y-0.2, min.z-0.2);
        max.set(max.x+0.2, max.y+0.2, max.z+0.2);

        for (let i = 0; i < numSpheres; i++) {
            let rand = null;
            if(i < 27){
                rand = [(           i      % 3) * 0.5,
                        (Math.floor(i / 3) % 3) * 0.5,
                        (Math.floor(i / 9) % 3) * 0.5];
                //console.log("Using Grid", rand);
            } else {
                rand = Quasirandom.randomValue(3);
            }
            let x = (rand[0] * (max.x - min.x)) + min.x;
            let y = (rand[1] * (max.y - min.y)) + min.y;
            let z = (rand[2] * (max.z - min.z)) + min.z;

            //console.log("Checking point", rand, x, y, z);

            let hitPointInfo = bvh.closestPointToPoint(this.tempVec.set(x, y, z));//.distanceTo(this.tempVec);

            //// get the face normal to determine if the distance should be positive or negative
            //let tri = new THREE.Triangle();
            //let faceIndex = hitPointInfo.faceIndex;
            //let i0 = geometry.index.getX( faceIndex * 3 + 0 );
            //let i1 = geometry.index.getX( faceIndex * 3 + 1 );
            //let i2 = geometry.index.getX( faceIndex * 3 + 2 );
            //tri.setFromAttributeAndIndices( geometry.attributes.position, i0, i1, i2 );
            //tri.getNormal( this.tempNormal );
            ////delta.subVectors( target.point, point );
            ////sdfTex.image.data[ index ] = this.tempNormal.dot( delta ) > 0.0 ? - dist : dist;

            ////// Check if the point is inside the mesh
            ////this.tempRay.set(this.tempVec.set(x, y, z), this.forward);
            ////const hit = bvh.raycastFirst( this.tempRay, THREE.DoubleSide );
            ////const isInside = hit && hit.face.normal.dot( this.tempRay.direction ) > 0.0;
            ////if (!isInside) { i--; continue; }

            ////let radius = bvh.closestPointToPoint(this.tempVec).distance;//.distanceTo(this.tempVec);

            ////bvh.closestPointToPoint(this.tempVec.set(hitPointInfo.point.x - (this.tempNormal.x * 0.05),
            ////                                         hitPointInfo.point.y - (this.tempNormal.y * 0.05),
            ////                                         hitPointInfo.point.z - (this.tempNormal.z * 0.05)), hitPointInfo);

            //// Check if the point is inside the mesh
            //let radius = 0.2;
            //this.tempRay.set(this.tempVec.set(hitPointInfo.point.x, hitPointInfo.point.y, hitPointInfo.point.z), this.tempNormal.negate());
            //const hit = bvh.raycastFirst( this.tempRay, THREE.DoubleSide, 0.0001, radius );
            ////console.log(hit);
            ////const isInside = hit && hit.face.normal.dot( this.tempRay.direction ) > 0.0;
            ////if (!isInside) { i--; continue; }

            //if (hit){
            //    this.tempVec.set((hitPointInfo.point.x + hit.point.x) * 0.5,
            //                     (hitPointInfo.point.y + hit.point.y) * 0.5,
            //                     (hitPointInfo.point.z + hit.point.z) * 0.5);
            //}else{
            //    this.tempVec.set(hitPointInfo.point.x + (this.tempNormal.x * radius),
            //                     hitPointInfo.point.y + (this.tempNormal.y * radius),
            //                     hitPointInfo.point.z + (this.tempNormal.z * radius));
            //}

            //bvh.closestPointToPoint(this.tempVec, hitPointInfo);
            //radius = hitPointInfo.distance;//Math.max(hitPointInfo.distance, 0.05);

            //this.tempRay.set(this.tempVec, this.forward);
            //const hit2 = bvh.raycastFirst( this.tempRay, THREE.DoubleSide );
            //const isInside = hit2 && hit2.face.normal.dot( this.tempRay.direction ) > 0.0;
            //if(!isInside){
                this.tempVec.set(hitPointInfo.point.x, hitPointInfo.point.y, hitPointInfo.point.z);
                let radius = 0.01;
            //}

            radius = Math.max(radius, 0.01);

            positions[i * 4    ] = this.tempVec.x;
            positions[i * 4 + 1] = this.tempVec.y;
            positions[i * 4 + 2] = this.tempVec.z;
            positions[i * 4 + 3] = radius;
        }
        console.log("Pack Spheres took", performance.now() - startTime, "ms");
        return positions;
    }

    /** Iteratively apply torque to the basis using Cross products (in place of SVD) 
     * https://animation.rwth-aachen.de/media/papers/2016-MIG-StableRotation.pdf
     * @param {THREE.Matrix3} A Covariance Matrix
     * @param {THREE.Quaternion} curQuaternion The current quaternion
     * @param {number} iterations The number of iterations to run the algorithm */
    quaternionTorqueDecomposition(A, curQuaternion, iterations = 9) {
        // Cancels out the momentum from the prior frame (Should I??)
        //curQuaternion.set(0, 0, 0, 1);

        let QuatBasis  = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)];
        let quatMatrix = new THREE.Matrix4().makeRotationFromQuaternion(curQuaternion);

        for (let iter = 0; iter < iterations; iter++) {
            quatMatrix.makeRotationFromQuaternion(curQuaternion);
            quatMatrix.extractBasis(QuatBasis[0], QuatBasis[1], QuatBasis[2]);

            let omegaDenom = Math.abs(
                QuatBasis[0].dot(A[0]) +
                QuatBasis[1].dot(A[1]) +
                QuatBasis[2].dot(A[2]) + 0.00000001);

            let omega =
                QuatBasis[0].clone().cross(A[0] ).add(
                QuatBasis[1].clone().cross(A[1])).add(
                QuatBasis[2].clone().cross(A[2])).divideScalar(omegaDenom);

            let w = omega.length();
            if (w < 0.00000001) { break; }
            curQuaternion.premultiply(new THREE.Quaternion().setFromAxisAngle(omega.normalize(), w));
            curQuaternion.normalize(); //Normalizes the Quaternion; critical for error suppression
        }
    }

    transposeMult(vec1, vec2, average1, average2, collided = []) {
        // Initialize Cross Covariance Matrix
        let covariance = [[0, 0, 0],
                          [0, 0, 0],
                          [0, 0, 0]];

        if (vec1.length !== vec2.length) {
            console.error("Transpose Multiply Error: Arrays must be the same length!");
            return covariance;
        }

        for (let i = 0; i < 3; i++) {                    // i is the row in this matrix
            for (let j = 0; j < 3; j++) {                // j is the column in the other matrix
                for (let k = 0; k < vec1.length; k+=4) { // k is the column in this matrix
                    let weight = 1.0;//(collided.includes(i) ? 10.0 : 1.0);

                    covariance[i][j] += (vec1[k + i] - average1[i]) * 
                                        (vec2[k + j] - average2[j]) * weight;
                }
            }
        }

        covariance = [new THREE.Vector3(covariance[0][0], covariance[0][1], covariance[0][2]),
                      new THREE.Vector3(covariance[1][0], covariance[1][1], covariance[1][2]),
                      new THREE.Vector3(covariance[2][0], covariance[2][1], covariance[2][2])];

        return covariance;
    }

    kabschPoints(pointsIn, pointsRef, translation, rotation, collided) {
        let  inAverage = this.getAverage(pointsIn , collided);
        let refAverage = this.getAverage(pointsRef, collided);

        // Calculate the optimal rotation
        this.quaternionTorqueDecomposition(this.transposeMult(pointsIn, pointsRef, inAverage, refAverage, collided), rotation, 9);

        translation.set(-inAverage[0], -inAverage[1], -inAverage[2])
                   .applyQuaternion(rotation)
                   .add(new THREE.Vector3(refAverage[0], refAverage[1], refAverage[2]));

    }

}

//Generic Quasirandom Number Generating Class
//http://extremelearning.com.au/unreasonable-effectiveness-of-quasirandom-sequences/
class Quasirandom {
    static currentSeed = 0;
    static alphas = {};//new Dictionary<int, float[]>();
  
    static phi(dimension) {
      let x = 1.0;
      for (let i = 0; i < 20; i++){
        x = x - (Math.pow(x, dimension + 1) - x - 1) / ((dimension + 1) * Math.pow(x, dimension) - 1);
      }
      console.log("Phi:", x);
      return x;
    }
  
    static bakeAlphas(dimensions) {
      let gamma = Quasirandom.phi(dimensions);
      let newAlphas = new Float32Array(dimensions);
      for (let i = 0; i < dimensions; i++) { newAlphas[i] = Math.pow(1.0 / gamma, i + 1) % 1.0; }
      Quasirandom.alphas[dimensions] = newAlphas;
      console.log(Quasirandom.alphas);
      return newAlphas;
    }
  
    static randomValue(dimensions = 3, seed = 0) {
      let currentAlphas;
      //if (!alphas.TryGetValue(dimensions, out currentAlphas)) { currentAlphas = bakeAlphas(dimensions); }
      if (dimensions in Quasirandom.alphas) {
        currentAlphas = Quasirandom.alphas[dimensions];
      } else { 
        currentAlphas = Quasirandom.bakeAlphas(dimensions);
      }
      if (seed != 0) { Quasirandom.currentSeed = seed; }
      //console.log(Quasirandom.currentSeed);
      return Quasirandom.value(currentAlphas, dimensions);
    }
  
    static value(currentAlphas, dimensions = 3) {
      let value = new Float32Array(dimensions);
      for (let i = 0; i < dimensions; i++) { value[i] = (Quasirandom.currentSeed * currentAlphas[i]) % 1.0; }
      Quasirandom.currentSeed += 1;
      return value;
    }
}
