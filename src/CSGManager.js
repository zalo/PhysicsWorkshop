import * as THREE from '../node_modules/three/build/three.module.js';
import Module /*, {Manifold}*/ from '../node_modules/manifold-3d/manifold.js';
import { mergeVertices, toCreasedNormals } from '../node_modules/three/examples/jsm/utils/BufferGeometryUtils.js';
import { SUBTRACTION, Brush, Evaluator, GridMaterial, HalfEdgeMap } from '../node_modules/three-bvh-csg/build/index.module.js';
//import { Dcel } from './Dcel.js';

// Initialize manifold outside of the class, which is otherwise just a collection of static functions.
const manifold = await Module();
manifold.setup();

/** Translate three.js Concepts into manifold-3d concepts */
class CSGManager {
    constructor() {
        this.initialized = false;
        this.setup();
    }
    async setup() {}

    /**
     * Converts a three.js mesh to a manifold-3d mesh.
     * @param {THREE.Mesh} threeMesh - The three.js mesh to convert. */
    static threeToManifold(threeMesh) {
        let geometry = threeMesh.geometry;
        if(threeMesh.geometry.manifoldGeometry != undefined){
            geometry = threeMesh.geometry.manifoldGeometry;
        }else{
            let mergedVertices = mergeVertices(threeMesh.geometry.clone(), 1e-3);
            geometry = mergedVertices;
        }
        let position = geometry.attributes.position.array;
        //let normal   = geometry.attributes.normal.array;
        let index    = geometry.index.array;

        let matrix = new THREE.Matrix4();
        matrix.compose(threeMesh.position, threeMesh.quaternion, threeMesh.scale);

        let transformedPosition = new Float32Array(position.length);
        for (let i = 0; i < position.length; i += 3) {
            let vertex = new THREE.Vector3(position[i], position[i + 1], position[i + 2]);
            vertex.applyMatrix4(matrix);
            transformedPosition[i] = vertex.x;
            transformedPosition[i + 1] = vertex.y;
            transformedPosition[i + 2] = vertex.z;

            //transformedPosition[i + 3] = normal[i];
            //transformedPosition[i + 4] = normal[i + 1];
            //transformedPosition[i + 5] = normal[i + 2];
        }

        let vertProperties = new Float32Array(transformedPosition.length);
        vertProperties.set(transformedPosition);

        let triVerts = new Uint32Array(index.length);
        triVerts.set(index);

        let meshOptions = {
            numProp: 3,
            vertProperties: vertProperties,
            triVerts: triVerts
        };

        return new manifold.Manifold(new manifold.Mesh(meshOptions));
    }

    /**
     * Converts a manifold-3d mesh to a three.js mesh.
     * @param {Manifold} manifoldMesh - The manifold-3d mesh to convert.
     * @param {THREE.Mesh} threeMesh - The original three.js mesh.
     * @returns {THREE.Mesh} - The resulting three.js mesh.
     */
    static manifoldToThree(manifoldMesh, threeMesh = null) {
        /** @type {Manifold[]} */
        let islands = manifoldMesh.decompose();

        let geometries = [];
        for(let m = 0; m < islands.length; m++){
            //let dummy = new manifold.Manifold();
            let volume   = islands[m].volume();
            let mesh     = islands[m].getMesh();
            let geometry = new THREE.BufferGeometry();
            let position = new Float32Array(mesh.vertProperties.length);// / 2);
            //let normal = new Float32Array(mesh.vertProperties.length / 2);

            if (threeMesh != null) {
                let matrix = new THREE.Matrix4();
                matrix.compose(threeMesh.position, threeMesh.quaternion, threeMesh.scale).invert();

                for (let i = 0; i < mesh.vertProperties.length; i += 3) {
                    let vertex = new THREE.Vector3(mesh.vertProperties[i], mesh.vertProperties[i + 1], mesh.vertProperties[i + 2]);
                    vertex.applyMatrix4(matrix);
                    position[i] = vertex.x;
                    position[i + 1] = vertex.y;
                    position[i + 2] = vertex.z;
                }
            }else{
                position.set(mesh.vertProperties);
            }

            if(position.length > 0) {
                geometry.setAttribute('position', new THREE.BufferAttribute(position, 3));
                //geometry.setAttribute('normal', new THREE.BufferAttribute(normal, 3));
                geometry.setIndex(new THREE.BufferAttribute(mesh.triVerts, 1));
                geometry.computeVertexNormals();

                let creasedGeometry = toCreasedNormals(geometry, 30 * 0.0174533); // Breaks manifoldness
                creasedGeometry.manifoldGeometry = geometry;
                if(creasedGeometry.userData == undefined){ creasedGeometry.userData = {}; }
                creasedGeometry.userData.volume = volume;
                geometries.push(creasedGeometry);
            }
        }

        // Sort meshes by volume
        geometries.sort((a, b) => b.userData.volume - a.userData.volume);

        let meshes = [];
        if (threeMesh === null) {
            let resultMesh = new THREE.Mesh(geometries[0], threeMesh != null ? threeMesh.material : new THREE.MeshPhysicalMaterial({ color: 0x00ff00, enableGrid: false }));
            meshes.push(resultMesh);
        } else {
            threeMesh.geometry = geometries[0];
            threeMesh.needsUpdate = true;
            meshes.push(threeMesh);
            for(let i = 1; i < geometries.length; i++){
                if(geometries[i] && geometries[i].attributes.position.array.length > 0){
                    let resultMesh = threeMesh.clone();
                    resultMesh.geometry = geometries[i];
                    meshes.push(resultMesh);
                }
            }
        }
        return meshes;
    }

    static createSphere(radius, segments) {
        let sphereManifold = new manifold.Manifold.sphere(radius, segments);
        return this.manifoldToThree(sphereManifold)[0];
    }

    static createBox(x, y, z, center = true) {
        let boxManifold = new manifold.Manifold.cube([x, y, z], center);
        return this.manifoldToThree(boxManifold)[0];
    }

    /**
     * Performs a boolean union operation on two three.js meshes.
     * @param {THREE.Mesh} meshA - The first three.js mesh.
     * @param {THREE.Mesh} meshB - The second three.js mesh.
     * @returns {THREE.Mesh} - The resulting three.js mesh after the union operation.
     */
    static union(meshA, meshB) {
        let startTime = performance.now();

        let manifoldA = CSGManager.threeToManifold(meshA);
        let manifoldB = CSGManager.threeToManifold(meshB);
        let resultManifold = manifoldA.add(manifoldB);
        manifoldA.delete();
        manifoldB.delete();
        let result = CSGManager.manifoldToThree(resultManifold, meshA);
        resultManifold.delete();

        console.log("Union took", performance.now() - startTime, "ms");
        return result;
    }

    /**
     * Performs a boolean subtract operation on two three.js meshes.
     * @param {THREE.Mesh} meshA - The first three.js mesh.
     * @param {THREE.Mesh} meshB - The second three.js mesh.
     * @returns {THREE.Mesh} - The resulting three.js mesh after the subtract operation.
     */
    static subtract(meshA, meshB) {
        let startTime = performance.now();

        let manifoldA = CSGManager.threeToManifold(meshA);
        let manifoldB = CSGManager.threeToManifold(meshB);
        let resultManifold = manifoldA.subtract(manifoldB);
        manifoldA.delete();
        manifoldB.delete();
        let result = CSGManager.manifoldToThree(resultManifold, meshA);
        resultManifold.delete();

        if (result[0].userData.isPhysicsObject) { result[0].userData.physics.needsUpdate = true; }

        console.log("Subtract took", performance.now() - startTime, "ms");
        return result;
    }

    /**
     * Performs a boolean subtract operation on two three.js meshes.
     * @param {THREE.Mesh} meshA - The first three.js mesh.
     * @param {THREE.Mesh} meshB - The second three.js mesh.
     * @returns {THREE.Mesh} - The resulting three.js mesh after the subtract operation.
     */
    static subtractBVH(meshA, meshB) {
        let startTime = performance.now();

        let brush1 = new Brush( meshA.geometry );
        brush1.position.copy( meshA.position );
        brush1.quaternion.copy( meshA.quaternion );
        brush1.scale.copy( meshA.scale );
        brush1.updateMatrixWorld();
        
        let brush2 = new Brush( meshB.geometry );
        brush2.position.copy( meshB.position );
        brush2.quaternion.copy( meshB.quaternion );
        brush2.scale.copy( meshB.scale );
        brush2.updateMatrixWorld();

        let evaluator = new Evaluator();
        evaluator.attributes = [ 'position', 'normal' ];
        let result = evaluator.evaluate( brush1, brush2, SUBTRACTION );

        meshA.geometry.dispose();
        meshA.geometry = result.geometry;

        //console.log("Triangles:", meshA.geometry.index);
        //let triangleToBodyIndex = new Uint8Array( meshA.geometry.index.array.length / 3 );
        //let mergedVertices = mergeVertices(meshA.geometry.clone(), 1e-3);

        //let index = []; for (let i = 0; i < index.length; i++) { index.push(i); }
        //meshA.geometry.setIndex(index);
        //let mergedVertices = mergeVertices(meshA.geometry.clone(), 1e-3);
        //let dcel = new Dcel(mergedVertices);
        //dcel.forAdjacentFaces(100, (adjacentFaceIndex) => {
        //    console.log("Adjacent Face:", adjacentFaceIndex);
        //});


        let matrix = new THREE.Matrix4();
        matrix.compose(meshA.position, meshA.quaternion, meshA.scale).invert();
        let invQuat = new THREE.Quaternion().copy(meshA.quaternion).invert();
        for (let i = 0; i < meshA.geometry.attributes.position.array.length; i += 3) {
            let vertex = new THREE.Vector3(
                meshA.geometry.attributes.position.array[i], 
                meshA.geometry.attributes.position.array[i + 1],
                meshA.geometry.attributes.position.array[i + 2]);
            vertex.applyMatrix4(matrix);
            meshA.geometry.attributes.position.array[i    ] = vertex.x;
            meshA.geometry.attributes.position.array[i + 1] = vertex.y;
            meshA.geometry.attributes.position.array[i + 2] = vertex.z;

            let normal = new THREE.Vector3(
                meshA.geometry.attributes.normal.array[i], 
                meshA.geometry.attributes.normal.array[i + 1],
                meshA.geometry.attributes.normal.array[i + 2]);
            normal.applyQuaternion(invQuat);
            meshA.geometry.attributes.normal.array[i    ] = normal.x;
            meshA.geometry.attributes.normal.array[i + 1] = normal.y;
            meshA.geometry.attributes.normal.array[i + 2] = normal.z;
        }

        if (meshA.userData.isPhysicsObject) { meshA.userData.physics.needsUpdate = true; }

        console.log("BVH Subtract took", performance.now() - startTime, "ms");
        return meshA;
    }

    /**
     * Performs a boolean intersect operation on two three.js meshes.
     * @param {THREE.Mesh} meshA - The first three.js mesh.
     * @param {THREE.Mesh} meshB - The second three.js mesh.
     * @returns {THREE.Mesh} - The resulting three.js mesh after the intersect operation.
     */
    static intersect(meshA, meshB) {
        let startTime = performance.now();

        let manifoldA = CSGManager.threeToManifold(meshA);
        let manifoldB = CSGManager.threeToManifold(meshB);
        let resultManifold = manifoldA.intersect(manifoldB);
        manifoldA.delete();
        manifoldB.delete();
        let result = CSGManager.manifoldToThree(resultManifold, meshA);
        resultManifold.delete();

        console.log("Intersect took", performance.now() - startTime, "ms");
        return result;
    }

}

export default CSGManager;
