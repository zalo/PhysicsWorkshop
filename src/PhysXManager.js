import * as THREE from '../node_modules/three/build/three.module.js';
import PhysX from '../assets/physx-js-webidl/dist/physx-js-webidl.mjs';

/** PhysX rigid body physics manager for Three.js meshes */
export default class PhysXManager {
    constructor() {
        /** @type {Map<THREE.Mesh, object>} mesh -> { actor, shapes[], triMesh, isStatic } */
        this.physicsObjects = new Map();
        this.initialized = false;
        this.lastTime = 0;
    }

    async setup(scene) {
        this.scene = scene;
        this.px = await PhysX({
            locateFile: (name) => `./assets/physx-js-webidl/dist/${name}`
        });

        let version = this.px.PHYSICS_VERSION;
        this.allocator = new this.px.PxDefaultAllocator();
        this.errorCb = new this.px.PxDefaultErrorCallback();
        this.foundation = this.px.CreateFoundation(version, this.allocator, this.errorCb);

        let tolerances = new this.px.PxTolerancesScale();
        this.tolerances = tolerances;
        this.physics = this.px.CreatePhysics(version, this.foundation, tolerances);

        let gravity = new this.px.PxVec3(0, -9.81, 0);
        let sceneDesc = new this.px.PxSceneDesc(tolerances);
        sceneDesc.set_gravity(gravity);
        sceneDesc.set_cpuDispatcher(this.px.DefaultCpuDispatcherCreate(0));
        sceneDesc.set_filterShader(this.px.DefaultFilterShader());
        this.pxScene = this.physics.createScene(sceneDesc);

        this.material = this.physics.createMaterial(0.6, 0.6, 0.3);
        this.filterData = new this.px.PxFilterData(1, 1, 0, 0);
        this.shapeFlags = new this.px.PxShapeFlags(
            this.px.eSCENE_QUERY_SHAPE |
            this.px.eSIMULATION_SHAPE
        );

        this.cookingParams = new this.px.PxCookingParams(tolerances);
        this.cookingParams.get_meshPreprocessParams().raise(this.px.eENABLE_INERTIA);

        this._createGroundPlane();

        console.log('PhysX loaded! Version: ' + ((version >> 24) & 0xff) + '.' + ((version >> 16) & 0xff) + '.' + ((version >> 8) & 0xff));
        this.initialized = true;
        return this.px;
    }

    _createGroundPlane() {
        let groundGeom = new this.px.PxBoxGeometry(10, 0.01, 10);
        let v = new this.px.PxVec3(0, -0.01, 0);
        let q = new this.px.PxQuat(0, 0, 0, 1);
        let pose = new this.px.PxTransform(v, q);
        let shape = this.physics.createShape(groundGeom, this.material, true, this.shapeFlags);
        shape.setSimulationFilterData(this.filterData);
        let actor = this.physics.createRigidStatic(pose);
        actor.attachShape(shape);
        this.pxScene.addActor(actor);
    }

    /** Cook a triangle mesh with SDF */
    _cookTriangleMeshSDF(vertices, indices) {
        let numVerts = vertices.length / 3;
        let numTris = indices.length / 3;
        if (numVerts < 4 || numTris < 1) return null;

        // Compute spacing from bounding box
        let minX = Infinity, minY = Infinity, minZ = Infinity;
        let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
        for (let i = 0; i < numVerts; i++) {
            let x = vertices[i * 3], y = vertices[i * 3 + 1], z = vertices[i * 3 + 2];
            minX = Math.min(minX, x); minY = Math.min(minY, y); minZ = Math.min(minZ, z);
            maxX = Math.max(maxX, x); maxY = Math.max(maxY, y); maxZ = Math.max(maxZ, z);
        }
        let longestAxis = Math.max(maxX - minX, maxY - minY, maxZ - minZ);
        let spacing = longestAxis / 16;

        let inputVerts = new this.px.PxArray_PxVec3(numVerts);
        for (let i = 0; i < numVerts; i++) {
            inputVerts.set(i, new this.px.PxVec3(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]));
        }
        let inputTris = new this.px.PxArray_PxU32(indices.length);
        for (let i = 0; i < indices.length; i++) {
            inputTris.set(i, indices[i]);
        }

        let pointsData = new this.px.PxBoundedData();
        pointsData.set_count(numVerts);
        pointsData.set_stride(12);
        pointsData.set_data(inputVerts.begin());

        let trisData = new this.px.PxBoundedData();
        trisData.set_count(numTris);
        trisData.set_stride(12);
        trisData.set_data(inputTris.begin());

        let desc = new this.px.PxTriangleMeshDesc();
        desc.set_points(pointsData);
        desc.set_triangles(trisData);

        let sdfDesc = new this.px.PxSDFDesc();
        sdfDesc.set_spacing(spacing);
        sdfDesc.set_subgridSize(0);
        sdfDesc.set_bitsPerSubgridPixel(this.px.e32_BIT_PER_PIXEL);
        sdfDesc.set_numThreadsForSdfConstruction(1);
        desc.set_sdfDesc(sdfDesc);

        let gridX = Math.ceil((maxX-minX)/spacing)+1, gridY = Math.ceil((maxY-minY)/spacing)+1, gridZ = Math.ceil((maxZ-minZ)/spacing)+1;
        let t0 = performance.now();
        let triMesh = this.px.CreateTriangleMesh(this.cookingParams, desc);
        console.log(`SDF bake: ${numTris} tris, ${gridX}x${gridY}x${gridZ}, ${(performance.now()-t0).toFixed(1)}ms`);

        inputVerts.__destroy__();
        inputTris.__destroy__();
        pointsData.__destroy__();
        trisData.__destroy__();
        desc.__destroy__();
        sdfDesc.__destroy__();

        return triMesh;
    }

    /** Create a PhysX rigid body for a Three.js mesh using SDF triangle mesh collision. */
    createPhysicsObject(mesh, isStatic = false, density = 1.0) {
        if (!mesh.geometry || !mesh.geometry.attributes?.position) {
            mesh.userData.isPhysicsObject = false;
            return null;
        }
        mesh.updateMatrixWorld();
        mesh.userData.isPhysicsObject = true;

        let physicsGeom = mesh.geometry.manifoldGeometry || mesh.geometry;
        let posAttr = physicsGeom.attributes.position;
        let localVerts = new Float32Array(posAttr.count * 3);
        let tmpV = new THREE.Vector3();
        for (let i = 0; i < posAttr.count; i++) {
            tmpV.fromBufferAttribute(posAttr, i);
            tmpV.multiply(mesh.scale);
            localVerts[i * 3] = tmpV.x;
            localVerts[i * 3 + 1] = tmpV.y;
            localVerts[i * 3 + 2] = tmpV.z;
        }

        let indices;
        if (physicsGeom.index) {
            indices = new Uint32Array(physicsGeom.index.array);
        } else {
            indices = new Uint32Array(posAttr.count);
            for (let i = 0; i < posAttr.count; i++) indices[i] = i;
        }

        let pos = mesh.position;
        let rot = mesh.quaternion;
        let v = new this.px.PxVec3(pos.x, pos.y, pos.z);
        let q = new this.px.PxQuat(rot.x, rot.y, rot.z, rot.w);
        let transform = new this.px.PxTransform(v, q);

        let triMesh = this._cookTriangleMeshSDF(localVerts, indices);
        let shape;
        if (triMesh) {
            let triGeom = new this.px.PxTriangleMeshGeometry(triMesh);
            shape = this.physics.createShape(triGeom, this.material, true, this.shapeFlags);
            console.log(`SDF triangle mesh: ${triMesh.getNbTriangles()} tris, SDF: ${triMesh.getSDF() !== 0}`);
        } else {
            console.warn('Triangle mesh cooking failed, using box fallback');
            let bbox = new THREE.Box3().setFromBufferAttribute(posAttr);
            let size = new THREE.Vector3();
            bbox.getSize(size);
            size.multiply(mesh.scale);
            let boxGeom = new this.px.PxBoxGeometry(
                Math.max(size.x / 2, 0.01), Math.max(size.y / 2, 0.01), Math.max(size.z / 2, 0.01)
            );
            shape = this.physics.createShape(boxGeom, this.material, true, this.shapeFlags);
        }
        shape.setSimulationFilterData(this.filterData);

        let actor = isStatic
            ? this.physics.createRigidStatic(transform)
            : this.physics.createRigidDynamic(transform);
        actor.attachShape(shape);

        if (!isStatic) {
            this.px.PxRigidBodyExt.prototype.updateMassAndInertia(actor, density);
            actor.setLinearDamping(0.2);
            actor.setAngularDamping(0.3);
            actor.setSolverIterationCounts(8, 4);
        }

        this.pxScene.addActor(actor);

        let entry = { actor, shapes: [shape], triMesh, isStatic };
        this.physicsObjects.set(mesh, entry);

        return actor;
    }

    /** Recreate physics body after geometry change. Preserves velocity. */
    recreatePhysicsObject(mesh) {
        let entry = this.physicsObjects.get(mesh);
        let linVel = null, angVel = null;

        if (entry) {
            if (!entry.isStatic) {
                let lv = entry.actor.getLinearVelocity();
                let av = entry.actor.getAngularVelocity();
                linVel = { x: lv.get_x(), y: lv.get_y(), z: lv.get_z() };
                angVel = { x: av.get_x(), y: av.get_y(), z: av.get_z() };
            }
            this.removePhysicsObject(mesh);
        }

        this.createPhysicsObject(mesh, false, 1.0);

        if (linVel) {
            let newEntry = this.physicsObjects.get(mesh);
            if (newEntry && !newEntry.isStatic) {
                newEntry.actor.setLinearVelocity(new this.px.PxVec3(linVel.x, linVel.y, linVel.z));
                newEntry.actor.setAngularVelocity(new this.px.PxVec3(angVel.x, angVel.y, angVel.z));
            }
        }
    }

    /** Remove a physics object */
    removePhysicsObject(mesh) {
        let entry = this.physicsObjects.get(mesh);
        if (!entry) return;

        this.pxScene.removeActor(entry.actor);
        if (entry.triMesh) entry.triMesh.release();
        entry.actor.release();
        this.physicsObjects.delete(mesh);
        mesh.userData.isPhysicsObject = false;
    }

    // ---------------------------------------------------------------------------
    // TODO: Worker-based hi-res SDF baking
    // Currently broken — cross-WASM-instance deserialization causes memory access
    // errors and simulation crashes after shape hot-swap.
    // See SDFCookWorker.js for the worker implementation.
    //
    // The approach: cook coarse SDF on main thread for instant collision, then cook
    // hi-res SDF in worker via CookTriangleMesh→PxDefaultMemoryOutputStream,
    // transfer bytes back, deserialize on main thread via
    // PxDefaultMemoryInputData→physics.createTriangleMesh, and hot-swap the shape.
    //
    // Issues to debug:
    //   1. HEAPU8.buffer may be detached after WASM memory growth — use HEAPU8[ptr+i]
    //   2. numThreadsForSdfConstruction must be 1 (WASM has no thread support)
    //   3. subgridSize must be 0 (sparse SDF also hangs, likely uses threads internally)
    //   4. updateMassAndInertia crashes on deserialized mesh — skip it (mass unchanged)
    //   5. getSDF() crashes on deserialized mesh — the mesh may be valid but vtable differs
    //   6. After hot-swap, simulation crashes with "null function" — possibly the
    //      deserialized mesh from a separate WASM PhysX instance isn't compatible
    //      with the main instance's scene, even though the cooked binary format
    //      should be portable.
    // ---------------------------------------------------------------------------
    //
    // _setupWorker() {
    //     this._nextRequestId = 0;
    //     this._pendingCooks = new Map();
    //     this.worker = new Worker(new URL('./SDFCookWorker.js', import.meta.url), { type: 'module' });
    //     this.worker.onmessage = (e) => {
    //         if (e.data.type === 'ready') { console.log('SDF cook worker ready'); return; }
    //         let { id, cookedData, bakeTimeMs, error } = e.data;
    //         if (error) { console.warn('SDF worker error:', error); return; }
    //         this._onWorkerResult(id, cookedData, bakeTimeMs);
    //     };
    // }
    //
    // _onWorkerResult(id, cookedData, bakeTimeMs) {
    //     let pending = this._pendingCooks.get(id);
    //     if (!pending) return;
    //     this._pendingCooks.delete(id);
    //     let { mesh, entry } = pending;
    //     if (!entry || !entry.actor) return;
    //
    //     // Deserialize cooked mesh on main thread
    //     let bytes = new Uint8Array(cookedData);
    //     let pxBuf = new this.px.PxArray_PxU8(bytes.length);
    //     let wasmPtr = pxBuf.begin();
    //     for (let i = 0; i < bytes.length; i++) {
    //         this.px.HEAPU8[wasmPtr + i] = bytes[i];
    //     }
    //     let inputData = new this.px.PxDefaultMemoryInputData(wasmPtr, bytes.length);
    //     let triMesh = this.physics.createTriangleMesh(inputData);
    //     inputData.__destroy__();
    //     pxBuf.__destroy__();
    //
    //     if (!triMesh) { console.warn('Failed to deserialize hi-res SDF mesh'); return; }
    //
    //     // Hot-swap the shape
    //     let actor = entry.actor;
    //     for (let shape of entry.shapes) actor.detachShape(shape);
    //     if (entry.triMesh) entry.triMesh.release();
    //
    //     let triGeom = new this.px.PxTriangleMeshGeometry(triMesh);
    //     let shape = this.physics.createShape(triGeom, this.material, true, this.shapeFlags);
    //     shape.setSimulationFilterData(this.filterData);
    //     actor.attachShape(shape);
    //
    //     entry.shapes = [shape];
    //     entry.triMesh = triMesh;
    //     // Mass/inertia already set from coarse mesh — same geometry, just finer SDF
    //
    //     console.log(`Hi-res SDF swapped in, worker bake: ${bakeTimeMs.toFixed(1)}ms`);
    // }
    //
    // // To kick off worker bake from createPhysicsObject, add after this.pxScene.addActor(actor):
    // //   this._setupWorker(); // call once in setup()
    // //   let hiResSpacing = longestAxis / 40;
    // //   let requestId = this._nextRequestId++;
    // //   this._pendingCooks.set(requestId, { mesh, entry });
    // //   this.worker.postMessage(
    // //       { id: requestId, vertices: new Float32Array(localVerts),
    // //         indices: new Uint32Array(indices), spacing: hiResSpacing },
    // //       [localVerts.buffer, indices.buffer]
    // //   );
    // ---------------------------------------------------------------------------

    /** Step physics and sync Three.js transforms */
    update() {
        if (!this.initialized) return;

        let now = performance.now();
        if (!this.lastTime) { this.lastTime = now; return; }

        let dt = Math.min((now - this.lastTime) / 1000, 0.05);
        this.lastTime = now;
        if (dt <= 0) return;

        let substep = 1 / 120;
        let remaining = dt;
        while (remaining > 0.0001) {
            let step = Math.min(remaining, substep);
            this.pxScene.simulate(step);
            this.pxScene.fetchResults(true);
            remaining -= step;
        }

        for (let [mesh, entry] of this.physicsObjects) {
            if (entry.isStatic) continue;
            let pose = entry.actor.getGlobalPose();
            let p = pose.get_p();
            let q = pose.get_q();
            mesh.position.set(p.get_x(), p.get_y(), p.get_z());
            mesh.quaternion.set(q.get_x(), q.get_y(), q.get_z(), q.get_w());

            if (p.get_y() < -5) {
                let resetV = new this.px.PxVec3(0, 2, 0);
                let resetQ = new this.px.PxQuat(0, 0, 0, 1);
                let resetPose = new this.px.PxTransform(resetV, resetQ);
                entry.actor.setGlobalPose(resetPose);
                entry.actor.setLinearVelocity(new this.px.PxVec3(0, 0, 0));
                entry.actor.setAngularVelocity(new this.px.PxVec3(0, 0, 0));
            }
        }
    }
}
