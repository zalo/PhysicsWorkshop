import * as THREE from 'three/webgpu';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import World from './World.js';
import CSGManager from './CSGManager.js';
import {
    SoftbodySimulation,
    tetrahedralize,
    processGeometry,
    processTetGeometry,
    PlaneCollider
} from 'tetrament';

/** The fundamental set up and animation structures for 3D Visualization */
export default class Main {

    constructor() {
        // Intercept Main Window Errors
        window.realConsoleError = console.error;
        window.addEventListener('error', (event) => {
            let path = event.filename.split("/");
            this.display((path[path.length - 1] + ":" + event.lineno + " - " + event.message));
        });
        console.error = this.fakeError.bind(this);

        /** @type {{ mesh: THREE.Mesh, instance: any, geometry: any, model: any }[]} */
        this.softbodies = [];
        this.simulation = null;
        this.rebaking = false;
        this.clock = new THREE.Clock();

        this.deferredConstructor();
    }

    async deferredConstructor() {
        // Configure Settings
        this.tetParams = {
            Resolution: 3,
            MinQuality: 0.001,
        };
        this.gui = new GUI();
        this.gui.add(this.tetParams, 'Resolution', 0, 20, 1);
        this.gui.add(this.tetParams, 'MinQuality', 0.0001, 0.1);

        // Construct the render world (WebGPU)
        this.world = new World(this);

        this.csg = new CSGManager(this);
        this.csg.setup();

        this.pointer = new THREE.Vector2();
        this.raycaster = new THREE.Raycaster();
        document.addEventListener('pointermove', this.onPointerMove.bind(this));
        document.addEventListener('pointerup', () => { this.onPointerUp(); });

        // Construct Test Shape
        this.objects = new THREE.Group();

        this.sphereMesh = CSGManager.createSphere(0.3, 22);
        this.sphereMesh.position.set(0.0, 1.0, 0.0);
        this.sphereMesh.material.transparent = true;
        this.sphereMesh.material.opacity = 0.5;
    }

    /** Called by World once WebGPU renderer is initialized */
    async onWorldReady() {
        this.world.scene.add(this.objects);
        this.world.scene.add(this.sphereMesh);

        // Create the softbody simulation with the WebGPU renderer
        this.simulation = new SoftbodySimulation(this.world.renderer, {
            stepsPerSecond: 180,
            gravity: new THREE.Vector3(0, -9.81, 0),
            damping: 0.999,
            friction: 0.3,
        });
        this.simulation.addCollider(PlaneCollider(new THREE.Vector3(0, 1, 0), 0));
        this.world.scene.add(this.simulation.object);

        // Create initial box
        let boxMesh = CSGManager.createBox(1, 1, 1, true);
        boxMesh.position.set(0.0, 1.0, 0.0);
        boxMesh.rotateOnAxis(new THREE.Vector3(1, 1, 1).normalize(), Math.PI / 4);
        this.objects.add(boxMesh);
        this.selectedObject = boxMesh;

        await this.addSoftbody(boxMesh);
    }

    /**
     * Tetrahedralizes a mesh's geometry and creates a softbody model for it.
     * @param {THREE.BufferGeometry} geometry - The surface geometry
     * @returns {Object} Model data ready for SoftbodySimulation.addGeometry
     */
    buildSoftbodyModel(geometry) {
        // Ensure the geometry has vertex normals and UVs
        geometry.computeVertexNormals();
        if (!geometry.getAttribute('uv')) {
            const count = geometry.getAttribute('position').count;
            geometry.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(count * 2), 2));
        }

        // Tetrahedralize the surface mesh (CDT)
        const { tetVerts, tetIds, vertices } = tetrahedralize(geometry, {
            resolution: this.tetParams.Resolution,
            minQuality: this.tetParams.MinQuality,
        });

        // Build tets array for surface vertex attachment
        const tets = [];
        for (let i = 0; i < tetIds.length; i += 4) {
            const v0 = vertices[tetIds[i]];
            const v1 = vertices[tetIds[i + 1]];
            const v2 = vertices[tetIds[i + 2]];
            const v3 = vertices[tetIds[i + 3]];
            const center = new THREE.Vector3()
                .add(v0).add(v1).add(v2).add(v3)
                .multiplyScalar(0.25);
            tets.push({ id: tets.length, v0, v1, v2, v3, center });
        }

        // Compute barycentric attachment of surface vertices to tetrahedra
        const surfaceData = processGeometry(geometry, tets);

        console.log(`[CDT] ${tetVerts.length / 3} tet verts, ${tetIds.length / 4} tets`);

        return {
            tetVerts: Array.from(tetVerts),
            tetIds: Array.from(tetIds),
            ...surfaceData,
            geometry
        };
    }

    /**
     * Adds a mesh as a softbody to the simulation and rebakes.
     * @param {THREE.Mesh} mesh
     */
    async addSoftbody(mesh) {
        // Get geometry in local space
        const geo = mesh.geometry.clone();
        geo.computeVertexNormals();

        const model = this.buildSoftbodyModel(geo);

        this.softbodies.push({
            mesh,
            model,
            spawnPosition: mesh.position.clone(),
            spawnQuaternion: mesh.quaternion.clone(),
            spawnScale: mesh.scale.clone(),
            meshOffset: new THREE.Vector3(), // spawnPosition - simRefPos (recomputed after each bake)
            instance: null,
            geometry: null,
        });

        await this.rebakeSimulation();
    }

    /**
     * Rebuilds the entire softbody simulation from the current softbodies list.
     * This is called after any CSG modification to re-tetrahedralize (CDT rebake).
     */
    async rebakeSimulation() {
        if (this.rebaking) return;
        this.rebaking = true;

        try {
            // Create a fresh simulation
            const oldSim = this.simulation;
            if (oldSim) {
                this.world.scene.remove(oldSim.object);
            }

            this.simulation = new SoftbodySimulation(this.world.renderer, {
                stepsPerSecond: 180,
                gravity: new THREE.Vector3(0, -2.0, 0),
                damping: 0.999,
                friction: 0.3,
            });
            this.simulation.addCollider(PlaneCollider(new THREE.Vector3(0, 1, 0), 0));
            this.world.scene.add(this.simulation.object);

            // Add all softbodies to the new simulation
            for (const body of this.softbodies) {
                const simGeometry = this.simulation.addGeometry(body.model);
                const instance = this.simulation.addInstance(simGeometry);
                body.geometry = simGeometry;
                body.instance = instance;
            }

            // Bake (compile GPU kernels)
            await this.simulation.bake();

            // Spawn all bodies at their spawn positions
            for (const body of this.softbodies) {
                await body.instance.spawn(
                    body.spawnPosition,
                    body.spawnQuaternion,
                    body.spawnScale,
                    new THREE.Vector3(0, 0, 0)
                );
            }

            // Readback positions to compute offset: meshOffset = spawnPos - simReadbackPos
            // This offset stays constant between rebakes and lets us derive mesh position
            // from the simulation readback: meshPos = simPos + meshOffset
            await this.simulation.readPositions();
            for (const body of this.softbodies) {
                const simPos = this.simulation.getPosition(body.instance.id);
                body.meshOffset.copy(body.spawnPosition).sub(simPos);
            }

            // Make original meshes invisible but still raycastable
            // (mesh.visible stays true so raycaster hits them;
            //  material.colorWrite/depthWrite false so they don't render)
            for (const body of this.softbodies) {
                if (!body.mesh.material._hiddenForRaycast) {
                    body.mesh.material = body.mesh.material.clone();
                    body.mesh.material.colorWrite = false;
                    body.mesh.material.depthWrite = false;
                    body.mesh.material._hiddenForRaycast = true;
                }
            }

            console.log(`[Rebake] ${this.softbodies.length} softbodies baked`);
        } finally {
            this.rebaking = false;
        }
    }

    /**
     * Handles CSG subtraction: modifies the mesh, re-tetrahedralizes, and rebakes.
     */
    async onPointerUp() {
        if (!this.selectedObject || this.rebaking) return;
        if (!this.simulation?.initialized) return;

        const targetBody = this.softbodies.find(b => b.mesh === this.selectedObject);
        if (!targetBody) return;

        // Force fresh readback and sync so mesh position is accurate before CSG
        await this.simulation.readPositions();
        for (const body of this.softbodies) {
            if (body.instance?.spawned) {
                const simPos = this.simulation.getPosition(body.instance.id);
                body.mesh.position.copy(simPos).add(body.meshOffset);
            }
        }

        // Perform CSG subtraction (operates in world space using mesh transforms)
        let meshes = CSGManager.subtract(this.selectedObject, this.sphereMesh);

        // Re-tetrahedralize the primary (modified) mesh - CDT rebake
        const newGeo = this.selectedObject.geometry.clone();
        newGeo.computeVertexNormals();
        targetBody.model = this.buildSoftbodyModel(newGeo);
        // Use mesh's current synced position for respawning (preserves position)
        targetBody.spawnPosition.copy(this.selectedObject.position);
        targetBody.spawnQuaternion.copy(this.selectedObject.quaternion);
        targetBody.spawnScale.copy(this.selectedObject.scale);

        // Handle fragments from the CSG operation
        if (meshes.length > 1) {
            for (let i = 1; i < meshes.length; i++) {
                const fragment = meshes[i];
                this.objects.add(fragment);

                const fragGeo = fragment.geometry.clone();
                fragGeo.computeVertexNormals();
                const fragModel = this.buildSoftbodyModel(fragGeo);

                this.softbodies.push({
                    mesh: fragment,
                    model: fragModel,
                    spawnPosition: fragment.position.clone(),
                    spawnQuaternion: fragment.quaternion.clone(),
                    spawnScale: fragment.scale.clone(),
                    meshOffset: new THREE.Vector3(),
                    instance: null,
                    geometry: null,
                });
            }
        }

        // Rebake the entire simulation with updated CDT data
        await this.rebakeSimulation();
    }

    onPointerMove(event) {
        if (this.pointer == undefined) { return; }
        this.pointer.x =   (event.clientX / window.innerWidth) * 2 - 1;
        this.pointer.y = - (event.clientY / window.innerHeight) * 2 + 1;
    }

    /** Update the simulation */
    update() {
        if (!this.world.initialized) return;

        this.world.controls.update();

        // Update softbody simulation
        const dt = this.clock.getDelta();
        if (this.simulation && this.simulation.initialized) {
            this.simulation.update(dt, this.clock.elapsedTime);

            // Sync source mesh positions: meshPos = simReadbackPos + meshOffset
            // meshOffset is recomputed after each rebake, so it's stable
            for (const body of this.softbodies) {
                if (body.instance?.spawned) {
                    const simPos = this.simulation.getPosition(body.instance.id);
                    body.mesh.position.copy(simPos).add(body.meshOffset);
                }
            }
        }

        // Raycast for CSG tool placement
        if (this.raycaster && this.simulation && this.simulation.initialized) {
            this.raycaster.setFromCamera(this.pointer, this.world.camera);
            let intersects = this.raycaster.intersectObjects(this.objects.children);
            if (intersects.length > 0 && intersects[0]) {
                let intersection = intersects[0];
                this.selectedObject = intersection.object;
                this.sphereMesh.position.copy(intersection.point);
            }
        }

        this.world.renderer.render(this.world.scene, this.world.camera);
        this.world.stats.update();
    }

    // Log Errors as <div>s over the main viewport
    fakeError(...args) {
        if (args.length > 0 && args[0]) { this.display(JSON.stringify(args[0])); }
        window.realConsoleError.apply(console, arguments);
    }

    display(text) {
        let errorNode = window.document.createElement("div");
        errorNode.innerHTML = text.fontcolor("red");
        window.document.getElementById("info").appendChild(errorNode);
    }
}

var main = new Main();
