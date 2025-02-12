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
        this.tempAvg        = new THREE.Vector3   (0, 0, 0);
        this.tempRay        = new THREE.Ray       (new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 1));
        this.tempMatrix     = new THREE.Matrix4  ();

        this.debugSpheres   = new THREE.InstancedBufferGeometry().copy(new THREE.SphereGeometry(1.0, 8, 8));
        this.debugMaterial  = new THREE.MeshBasicMaterial({ color: 0xaa0000 });
        this.debugMesh      = new THREE.InstancedMesh(this.debugSpheres, this.debugMaterial, 100);
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
        mesh.geometry.computeBoundingBox();
        if (!mesh.geometry.boundsTree) {
            mesh.geometry.computeBoundsTree();
        }

        let physicsReferencePositions = this.packSpheres(mesh.geometry, 100);

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

        this.physicsObjects.push(mesh);
    }

    /** Update all the physics objects by integrating the positions and updating the meshes  */
    update() {
        for (let object of this.physicsObjects) {
            if (object.userData.isPhysicsObject) {
                if(object.userData.physics.needsUpdate) {
                    object.userData.physics.needsUpdate = false;

                    if(!object.geometry){ object.userData.isPhysicsObject = false; continue; }

                    // Construct a bounding volume hierarchy for the mesh
                    object.geometry.computeBoundingBox();
                    if (!object.geometry.boundsTree) {
                        object.geometry.computeBoundsTree();
                    }

                    object.userData.physics.referencePositions = this.packSpheres(object.geometry, 100);
                    object.userData.physics.positions         = new Float32Array(object.userData.physics.referencePositions);
                    object.userData.physics.previousPositions = new Float32Array(object.userData.physics.referencePositions);
            
                    console.log("MESH UPDATED! Vertices:", object.userData.physics.positions.length/4);
                    this.debugMesh.instanceMatrix.needsUpdate = true;

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
                        this.debugMesh.setMatrixAt(i/4, this.tempMatrix.compose(this.tempVec, this.curQuaternion, this.tempVec2.set(scale, scale, scale)));
                    }
                }

                // Integrate the positions forward in time
                this.verletIntegrate(object.userData.physics.positions, object.userData.physics.previousPositions);

                // Add Gravity and Collide the positions against the ground
                let collided = [];
                for (let i = 0; i < object.userData.physics.positions.length; i+=4) {

                    //// Clamp distance from the origin
                    //let distance = Math.sqrt(object.userData.physics.positions[i  ] * object.userData.physics.positions[i  ] +
                    //                         object.userData.physics.positions[i+1] * object.userData.physics.positions[i+1] +
                    //                         object.userData.physics.positions[i+2] * object.userData.physics.positions[i+2]);
                    //if (distance > 3) {
                    //    object.userData.physics.positions[i  ] *= 3 / distance;
                    //    object.userData.physics.positions[i+1] *= 3 / distance;
                    //    object.userData.physics.positions[i+2] *= 3 / distance;
                    //}


                    object.userData.physics.positions[i+1] -= 0.001; // Gravity
                    if (object.userData.physics.positions[i+1] < object.userData.physics.positions[i+3]) {
                        collided.push(i);
                        object.userData.physics.positions[i+1] = object.userData.physics.positions[i+3];
                        // Friction - Mostly just horizontal damping while on the ground
                        let perpendicularVelocityX = object.userData.physics.positions[i]   - object.userData.physics.previousPositions[i];
                        let perpendicularVelocityZ = object.userData.physics.positions[i+2] - object.userData.physics.previousPositions[i+2];
                        object.userData.physics.positions[i]   -= perpendicularVelocityX * 0.25;
                        object.userData.physics.positions[i+2] -= perpendicularVelocityZ * 0.25;
                    }
                }

                // Kabsch the mesh vertices to the current positions
                this.kabschPoints(
                    object.userData.physics.referencePositions, 
                    object.userData.physics.positions,
                    object.position, object.quaternion, collided);

                object.updateMatrixWorld(); // Ensure the world matrix is up to date

                // Transform the positions to world space
                for(let i = 0; i < object.userData.physics.positions.length; i+=4) {
                    object.localToWorld(this.tempVec.fromArray(object.userData.physics.referencePositions, i));
                    object.userData.physics.positions[i]   = this.tempVec.x;
                    object.userData.physics.positions[i+1] = this.tempVec.y;
                    object.userData.physics.positions[i+2] = this.tempVec.z;

                    let scale = object.userData.physics.positions[i+3];
                    this.debugMesh.setMatrixAt(i/4, this.tempMatrix.compose(this.tempVec, this.curQuaternion, this.tempVec2.set(scale, scale, scale)));
                }
                this.debugMesh.instanceMatrix.needsUpdate = true;
            } else {
                //console.error("How did this get here?  Create physics objects with PhysicsManager.createPhysicsObject()!", object);
                // Skip for empty bodies...
            }
        }
    }


    /** Criminally simple verlet integration
     * XPBD is a more advanced version that handles elasticity in a more principles fashion */
    verletIntegrate(curPoints, pastPoints) {
        for (let i = 0; i < curPoints.length; i++) {
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
        let positions = new Float32Array(numSpheres * 4);

        /** @type {MeshBVH} */
        let bvh = geometry.boundsTree;
        /** @type {THREE.Box3} */
        let bounds = geometry.boundingBox;
        let min    = bounds.min;
        let max    = bounds.max;

        for (let i = 0; i < numSpheres; i++) {
            let rand = Quasirandom.randomValue(3);
            let x = (rand[0] * (max.x - min.x)) + min.x;
            let y = (rand[1] * (max.y - min.y)) + min.y;
            let z = (rand[2] * (max.z - min.z)) + min.z;

            //console.log("Checking point", rand, x, y, z);

            // Check if the point is inside the mesh
            this.tempRay.set(this.tempVec.set(x, y, z), this.forward);
            const hit = geometry.boundsTree.raycastFirst( this.tempRay, THREE.DoubleSide );
            const isInside = hit && hit.face.normal.dot( this.tempRay.direction ) > 0.0;
            if (!isInside) { i--; continue; }

            let radius = bvh.closestPointToPoint(this.tempVec).distance;//.distanceTo(this.tempVec);

            positions[i * 4    ] = x;
            positions[i * 4 + 1] = y;
            positions[i * 4 + 2] = z;
            positions[i * 4 + 3] = radius;
        }
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
      console.log(Quasirandom.currentSeed);
      return Quasirandom.value(currentAlphas, dimensions);
    }
  
    static value(currentAlphas, dimensions = 3) {
      let value = new Float32Array(dimensions);
      for (let i = 0; i < dimensions; i++) { value[i] = (Quasirandom.currentSeed * currentAlphas[i]) % 1.0; }
      Quasirandom.currentSeed += 1;
      return value;
    }
}
