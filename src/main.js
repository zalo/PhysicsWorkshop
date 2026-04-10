import * as THREE from '../node_modules/three/build/three.module.js';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import World from './World.js';
import { OBJLoader } from '../node_modules/three/examples/jsm/loaders/OBJLoader.js';
import PhysXManager from './PhysXManager.js';
import CSGManager from './CSGManager.js';

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
        this.deferredConstructor();
    }
    async deferredConstructor() {
        this.gui = new GUI();

        // Construct the render world
        this.world = new World(this);

        // Initialize PhysX physics
        this.physics = new PhysXManager();
        this.px = await this.physics.setup(this.world.scene);

        this.csg = new CSGManager(this);
        this.csg.setup();

        this.pointer = new THREE.Vector2();
        this.raycaster = new THREE.Raycaster();
        this.hasIntersection = false;
        document.addEventListener( 'pointermove', this.onPointerMove.bind(this) );
        document.addEventListener( 'pointerdown', ()=>{
            if(!this.hasIntersection || !this.selectedObject || !this.selectedObject.geometry) return;
            let meshes;
            try {
                meshes = CSGManager.subtract(this.selectedObject, this.sphereMesh);
            } catch(e) {
                console.warn('Subtract failed:', e.message);
                return;
            }
            // The primary mesh (meshes[0]) was modified in-place; recreate its physics body
            if(meshes[0] && meshes[0].geometry && meshes[0].userData.isPhysicsObject){
                this.physics.recreatePhysicsObject(meshes[0]);
            } else if(meshes[0] && !meshes[0].geometry){
                this.physics.removePhysicsObject(meshes[0]);
                this.objects.remove(meshes[0]);
            }
            // Any new fragment meshes get their own physics bodies
            for(let i = 1; i < meshes.length; i++){
                this.objects.add(meshes[i]);
                this.physics.createPhysicsObject(meshes[i]);
            }
        });

        // Construct Test Shape
        this.objects = new THREE.Group();
        this.world.scene.add(this.objects);
        this.boxMesh = CSGManager.createBox(1, 1, 1, true);
        this.boxMesh.position.set(0.0, 1.0, 0.0);
        this.boxMesh.rotateOnAxis(new THREE.Vector3(1, 1, 1).normalize(), Math.PI / 4);
        this.objects.add(this.boxMesh);
        this.selectedObject = this.boxMesh;
        this.sphereMesh = CSGManager.createSphere(0.3, 10);
        this.sphereMesh.position.set(0.0, 1.0, 0.0);
        this.sphereMesh.material.transparent = true;
        this.sphereMesh.material.opacity = 0.5;
        this.world.scene.add(this.sphereMesh);

        this.physics.createPhysicsObject(this.boxMesh);
    }

    onPointerMove( event ) {
        if(this.pointer == undefined){ return; }
        this.pointer.x =   ( event.clientX / window.innerWidth  ) * 2 - 1;
        this.pointer.y = - ( event.clientY / window.innerHeight ) * 2 + 1;
    }

    /** Update the simulation */
    update() {
        this.world.controls.update();

        if (!this.physics) { this.world.renderer.render(this.world.scene, this.world.camera); return; }
        this.physics.update();

        if (!this.objects) { this.world.renderer.render(this.world.scene, this.world.camera); return; }

        // Remove dead objects
        for (let i = this.objects.children.length - 1; i >= 0; i--) {
            let object = this.objects.children[i];
            if (object.userData.isPhysicsObject === false || !object.geometry) {
                this.objects.remove(object);
            }
        }

        if(this.raycaster){
            this.raycaster.setFromCamera( this.pointer, this.world.camera );
            let intersects = this.raycaster.intersectObjects(this.objects.children);
            if (intersects.length > 0 && intersects[0]) {
                let intersection = intersects[0];
                this.selectedObject = intersection.object;
                this.sphereMesh.position.copy(intersection.point);
                this.sphereMesh.visible = true;
                this.hasIntersection = true;
            } else {
                this.sphereMesh.visible = false;
                this.hasIntersection = false;
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
