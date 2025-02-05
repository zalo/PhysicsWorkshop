import * as THREE from '../node_modules/three/build/three.module.js';
import PhysX from '../assets/physx-js-webidl/dist/physx-js-webidl.js';

/** Translate three.js concepts back and forth with PhysX concepts */
export default class PhysXManager {
    constructor() {
        this.physicsObjects = new Map(); // Map Three.js objects to PhysX objects
        this.initialized = false;
    }

    async setup(scene) {
        this.scene = scene;
        this.px = await PhysX();
        let version = this.px.PHYSICS_VERSION;
        this.allocator = new this.px.PxDefaultAllocator();
        this.errorCb = new this.px.PxDefaultErrorCallback();
        this.foundation = this.px.CreateFoundation(version, this.allocator, this.errorCb);
        
        // Create physics instance
        let tolerances = new this.px.PxTolerancesScale();
        this.physics = this.px.CreatePhysics(version, this.foundation, tolerances);
        
        // Create scene with proper collision settings
        var tmpVec = new this.px.PxVec3(0, -9.81, 0);
        var sceneDesc = new this.px.PxSceneDesc(tolerances);
        sceneDesc.set_gravity(tmpVec);
        sceneDesc.set_cpuDispatcher(this.px.DefaultCpuDispatcherCreate(0));
        sceneDesc.set_filterShader(this.px.DefaultFilterShader());
        this.physicsScene = this.physics.createScene(sceneDesc);
        console.log('Created scene');

        // Setup collision filtering
        this.tmpFilterData = new this.px.PxFilterData(
            1,  // word0: collision group
            1,  // word1: collision mask (objects collide if (group1 & mask2) && (group2 & mask1))
            0,  // word2: reserved
            0   // word3: reserved
        );
        
        // Create default material with better friction and restitution
        this.material = this.physics.createMaterial(
            0.6,    // static friction
            0.6,    // dynamic friction
            0.2     // restitution (bounciness)
        );

        // Enable collision, scene query and visualization
        this.shapeFlags = new this.px.PxShapeFlags(
            this.px.PxShapeFlagEnum.eSCENE_QUERY_SHAPE | 
            this.px.PxShapeFlagEnum.eSIMULATION_SHAPE |
            this.px.PxShapeFlagEnum.eVISUALIZATION
        );

        this.tmpPose = new this.px.PxTransform(this.px.PxIDENTITYEnum.PxIdentity);

        console.log('PhysX loaded! Version: ' + ((version >> 24) & 0xff) + '.' + ((version >> 16) & 0xff) + '.' + ((version >> 8) & 0xff));
        this.initialized = true;
        return this.px;
    }

    createPhysicsBody(threeMesh, isStatic = false, density = 1.0) {
        const geometry = threeMesh.geometry;
        const position = threeMesh.position;
        const quaternion = threeMesh.quaternion;
        const scale = threeMesh.scale;

        // Create transform
        let transform = new this.px.PxTransform(
            new this.px.PxVec3(position.x, position.y, position.z),
            new this.px.PxQuat(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        );

        // Create shape based on geometry type
        let shape;
        if (geometry instanceof THREE.BoxGeometry) {
            const halfExtents = new this.px.PxVec3(
                geometry.parameters.width * scale.x / 2,
                geometry.parameters.height * scale.y / 2,
                geometry.parameters.depth * scale.z / 2
            );
            const boxGeom = new this.px.PxBoxGeometry(halfExtents);
            shape = this.physics.createShape(boxGeom, this.material, true, this.shapeFlags);
        } else if (geometry instanceof THREE.SphereGeometry) {
            const sphereGeom = new this.px.PxSphereGeometry(geometry.parameters.radius * scale.x);
            shape = this.physics.createShape(sphereGeom, this.material, true, this.shapeFlags);
        } else if (geometry instanceof THREE.PlaneGeometry) {
            const planeGeom = new this.px.PxPlaneGeometry();
            shape = this.physics.createShape(planeGeom, this.material, true, this.shapeFlags);
        /*} else if (geometry instanceof THREE.BufferGeometry) {
            // Create px.ConvexMeshGeometry from buffer geometry
            const inputStream = new this.px.PxInputStream(this.createSimpleMesh(geometry));
            const convexMesh = this.px.PxPhysics.prototype.createConvexMesh(inputStream);
            const bufferGeom = new this.px.PxConvexMeshGeometry(convexMesh);
            shape = this.physics.createShape(bufferGeom, this.material, true);*/
        } else {
            console.warn('Unsupported geometry type:', geometry);
            return null;
        }

        // Create rigid body
        let actor;
        if (isStatic) {
            actor = this.physics.createRigidStatic(transform);
        } else {
            actor = this.physics.createRigidDynamic(transform);
            
            // Set mass properties based on density
            this.px.PxRigidBodyExt.prototype.updateMassAndInertia(actor, density);
            
            // Enable CCD (Continuous Collision Detection) for better collision at high speeds
            actor.setRigidBodyFlag(this.px.PxRigidBodyFlagEnum.eENABLE_CCD, true);
            
            // Set reasonable default values for better simulation
            actor.setAngularDamping(0.5);
            actor.setLinearDamping(0.1);
            actor.setSolverIterationCounts(8, 2); // position iterations, velocity iterations
        }

        // Attach shape with proper collision filtering
        shape.setSimulationFilterData(this.tmpFilterData);
        actor.attachShape(shape);

        // Add to scene
        this.physicsScene.addActor(actor);
        
        // Store mapping
        this.physicsObjects.set(threeMesh, actor);
        
        return actor;
    }

    createSimpleMesh(geometry) {
                // First need to get the data into PhysX
                let vertices = geometry.attributes.position.array;
                let indices = geometry.index.array;
                let inputVertices = new this.px.PxArray_PxVec3(vertices.length/3);
                let inputIndices  = new this.px.PxArray_PxU32 (indices.length);

                for(let i = 0; i < vertices.length; i+=3){
                    inputVertices.set(i/3, new this.px.PxVec3(vertices[i], vertices[i+1], vertices[i+2]));
                }
                for(let i = 0; i < indices.length; i++){
                    inputIndices.set(i, indices[i]);
                    if(indices[i] < 0 || indices[i] >= inputVertices.size()){
                        console.log("Index out of range!", i, indices[i], inputVertices.size());
                    }
                }
        
                // Next need to make the PxBoundedData for both the vertices and indices to make the 'Simple'TriangleMesh
                let vertexData = new this.px.PxBoundedData();
                let indexData  = new this.px.PxBoundedData();
                vertexData.set_count(inputVertices.size ());
                vertexData.set_data (inputVertices.begin());
                indexData .set_count(inputIndices .size ()/3);
                indexData .set_data (inputIndices .begin());
                let simpleMesh = new this.px.PxSimpleTriangleMesh();
                simpleMesh.set_points   (vertexData);
                simpleMesh.set_triangles( indexData);
                return simpleMesh;
    }

    update() {
        if (!this.lastTime) {
            this.lastTime = performance.now();
            return;
        }

        const now = performance.now();
        const dt = Math.min((now - this.lastTime) / 1000, 1/30); // Cap at 1/30 sec to prevent large time steps
        this.lastTime = now;

        // Step physics simulation
        this.physicsScene.simulate(dt);
        this.physicsScene.fetchResults(true);

        // Update Three.js objects
        for (let [threeMesh, physxActor] of this.physicsObjects) {
            const transform = physxActor.getGlobalPose();
            const position = transform.p;
            const rotation = transform.q;

            threeMesh.position.set(position.x, position.y, position.z);
            threeMesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
        }
    }

    cleanup() {
        // Release all PhysX resources
        for (let [_, actor] of this.physicsObjects) {
            this.physicsScene.removeActor(actor);
            actor.release();
        }
        this.physicsObjects.clear();
        
        if (this.physicsScene) this.physicsScene.release();
        if (this.physics) this.physics.release();
        if (this.foundation) this.foundation.release();
    }
}