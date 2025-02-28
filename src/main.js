import * as THREE from '../node_modules/three/build/three.module.js';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import World from './World.js';
import { OBJLoader } from '../node_modules/three/examples/jsm/loaders/OBJLoader.js';
//import ManifoldModule from '../node_modules/manifold-3d/manifold.js';
//import PhysX from './PhysXManager.js';
//import PhysXManager from './PhysXManager.js';
import PhysicsManager from './PhysicsManager.js';
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
        this.physicsScene = { softBodies: [] };
        this.deferredConstructor();
    }
    async deferredConstructor() {
        // Configure Settings
        this.meshingParams = {
            RemeshResolution : 20,
            TargetTriangles  : 2000,
            MaxTriangleEdgeLength: 50.0,
            MinTetVolume: 0.01,
            ShowTetMesh      : true,
        };
        this.gui = new GUI();
        this.gui.add(this.meshingParams, 'RemeshResolution', 0, 50, 1).onFinishChange((value) => {
            if(this.mesh){ this.generateTetMesh(this.mesh); }});
        this.gui.add(this.meshingParams, 'TargetTriangles', 100, 5000, 100).onFinishChange((value) => {
            if(this.mesh){ this.generateTetMesh(this.mesh); }});
        this.gui.add(this.meshingParams, 'MaxTriangleEdgeLength').onFinishChange((value) => {
            if(this.mesh){ this.generateTetMesh(this.mesh); }});
        this.gui.add(this.meshingParams, 'MinTetVolume').onFinishChange((value) => {
            if(this.mesh){ this.generateTetMesh(this.mesh); }});

        // Construct the render world
        this.world = new World(this);

        //this.PhysX = new PhysXManager(this);
        //this.px = await this.PhysX.setup(this.world.scene);

        this.physics = new PhysicsManager(this.world.scene);

        this.csg = new CSGManager(this);
        this.csg.setup();

        this.pointer = new THREE.Vector2();
        this.raycaster = new THREE.Raycaster();
        document.addEventListener( 'pointermove', this.onPointerMove.bind(this) );
        document.addEventListener( 'pointerdown', ()=>{
            let meshes = CSGManager.subtract(this.selectedObject, this.sphereMesh);
            if(meshes.length > 1){
                for(let i = 1; i < meshes.length; i++){
                    this.objects.add(meshes[i]);
                    this.physics.createPhysicsObject(meshes[i]);
                }
            }
        });

        // Construct Test Shape
        this.objects = new THREE.Group();
        this.world.scene.add(this.objects);
        this.boxMesh = CSGManager.createBox(1, 1, 1, true);//createSphere(0.5, 32);//
        this.boxMesh.position.set(0.0, 1.0, 0.0);
        this.boxMesh.rotateOnAxis(new THREE.Vector3(1, 1, 1).normalize(), Math.PI / 4);
        this.objects.add(this.boxMesh);
        this.selectedObject = this.boxMesh;
        this.sphereMesh = CSGManager.createSphere(0.3, 22);
        this.sphereMesh.position.set(0.0, 1.0, 0.0);
        this.sphereMesh.material.transparent = true;
        this.sphereMesh.material.opacity = 0.5;
        this.world.scene.add(this.sphereMesh);

        //let spherelessBox = CSGManager.subtract(this.boxMesh, this.sphereMesh);
        //this.sphereMesh.position.set(0.5, 1.0, 0.0);
        //spherelessBox = CSGManager.subtract(spherelessBox, this.sphereMesh);

        this.physics.createPhysicsObject(this.boxMesh);

        /*let geometry = new THREE.BoxGeometry(1.0, 1.0, 1.0);
        let mesh = new THREE.Mesh(geometry, new THREE.MeshPhysicalMaterial({ color: 0x00ff00, wireframe: true }));
        mesh.position.set(0.0, 1.0, 0.0);
        this.world.scene.add(mesh);
        this.PhysX.createPhysicsBody(mesh, false, 1.0);

        let mesh2 = new THREE.Mesh(geometry, new THREE.MeshPhysicalMaterial({ color: 0x00ff00, wireframe: true }));
        mesh2.position.set(0.0, -0.5, 0.0);
        mesh2.scale.set(10.0, 1.0, 10.0);
        this.world.scene.add(mesh2);
        this.PhysX.createPhysicsBody(mesh2, true, 1.0);*/


        // load a resource
        /*new OBJLoader().load( './assets/armadillo.obj',
            ( object ) => { this.generateTetMesh(object.children[0]); },
            ( xhr    ) => { console.log( ( xhr.loaded / xhr.total * 100 ) + '% loaded' ); },
            ( error  ) => { console.log( 'A loading error happened', error );  }
        );*/
    }

    onPointerMove( event ) {
        if(this.pointer == undefined){ return; }
        this.pointer.x =   ( event.clientX / window.innerWidth  ) * 2 - 1;
        this.pointer.y = - ( event.clientY / window.innerHeight ) * 2 + 1;
    }

    /** Update the simulation */
    update() {
        // Render the scene and update the framerate counter
        this.world.controls.update();

        this.physics.update();

        this.objects.children.forEach((object) => {
            if(!object.userData.isPhysicsObject){
                this.objects.remove(object);
            }
        });

        if(this.raycaster){
            this.raycaster.setFromCamera( this.pointer, this.world.camera );
            let intersects = this.raycaster.intersectObjects(this.objects.children);
            if (intersects.length > 0 && intersects[0]) {
            /** @type {THREE.Intersection} */
            let intersection = intersects[0];
            this.selectedObject = intersection.object;
            //console.log(intersection);
            this.sphereMesh.position.copy(intersection.point);
            }
        }

        //if(this.PhysX.initialized){ this.PhysX.update(); }
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

    generateTetMesh(mesh){
        if(this.mesh){
            this.world.scene.remove(this.mesh);
            this.mesh.geometry.dispose();
            this.mesh.material.dispose();
            this.world.scene.remove(this.tetMesh);
            this.tetMesh.geometry.dispose();
            this.tetMesh.material.dispose();
        }

        this.mesh = mesh;
        this.mesh.material = new THREE.MeshPhysicalMaterial({ color: 0xf78a1d });
        this.centerMesh(this.mesh);
        this.world.scene.add( this.mesh ); 
        let index = this.mesh.geometry.getIndex();
        if(index == null){
            index = new THREE.BufferAttribute(new Uint32Array(this.mesh.geometry.getAttribute("position").array.length / 3), 1);
            for(let i = 0; i < index.count; i++){ index.array[i] = i; }
        }
    
        let remeshedGeo   = this.remesh(this.mesh.geometry.getAttribute("position").array, index.array, this.meshingParams.RemeshResolution);
        let simplifiedGeo = this.simplifyMesh(remeshedGeo.getAttribute("position").array, remeshedGeo.getIndex().array, this.meshingParams.TargetTriangles, this.meshingParams.MaxTriangleEdgeLength);
        //let remeshedThreeMesh = new THREE.Mesh(simplifiedGeo, new THREE.MeshPhysicalMaterial({ color: 0x00ff00, wireframe: true }));
        //remeshedThreeMesh.position.copy(this.mesh.position).add(new THREE.Vector3(0.0, 0.0, 0.0));
        //remeshedThreeMesh.scale.copy(this.mesh.scale);
        //this.world.scene.add(remeshedThreeMesh);

        let tetrahedronGeo = this.createConformingTetrahedronMesh(simplifiedGeo.getAttribute("position").array, simplifiedGeo.getIndex().array, this.meshingParams.MinTetVolume);
        this.tetMesh = new THREE.LineSegments(tetrahedronGeo.edgeMesh, new THREE.LineBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide }));
        this.tetMesh.userData = this;    // for raycasting
        this.tetMesh.layers.enable(1);
        this.tetMesh.visible = true;
        this.tetMesh.position.copy(this.mesh.position).add(new THREE.Vector3(0.0, 0.0, 0.0));
        this.tetMesh.scale.copy(this.mesh.scale);
        this.world.scene.add(this.tetMesh);
    }

    /** @returns {THREE.BufferGeometry} */
    remesh(vertices, indices, remesherGridResolution = 20){
        let inputVertices  = new this.px.PxArray_PxVec3(vertices.length/3);
        let inputIndices   = new this.px.PxArray_PxU32 (indices.length);
        for(let i = 0; i < vertices.length; i+=3){
            inputVertices.set(i/3, new this.px.PxVec3(vertices[i], vertices[i+1], vertices[i+2]));
        }
        for(let i = 0; i < indices.length; i++){
            inputIndices.set(i, indices[i]);
        }

        let outputVertices = new this.px.PxArray_PxVec3();
        let outputIndices  = new this.px.PxArray_PxU32 ();
        let vertexMap      = new this.px.PxArray_PxU32 ();
        this.px.PxTetMaker.prototype.remeshTriangleMesh(inputVertices, inputIndices, remesherGridResolution, outputVertices, outputIndices, vertexMap);

        // Transform From PxVec3 to THREE.Vector3
        let triIndices = new Uint32Array(outputIndices.size());
        for(let i = 0; i < triIndices.length; i++){
            triIndices[i] = outputIndices.get(i);
        }
        let vertPositions = new Float32Array(outputVertices.size() * 3);
        for(let i = 0; i < outputVertices.size(); i++){
            let vec3 = outputVertices.get(i);
            vertPositions[i*3+0] = vec3.get_x();
            vertPositions[i*3+1] = vec3.get_y();
            vertPositions[i*3+2] = vec3.get_z();
        }
        let remeshedBufferGeo = new THREE.BufferGeometry();
        remeshedBufferGeo.setAttribute('position', new THREE.BufferAttribute(vertPositions, 3));
        remeshedBufferGeo.setIndex(new THREE.BufferAttribute(triIndices, 1));
        remeshedBufferGeo.computeVertexNormals();
        inputVertices .__destroy__();
        inputIndices  .__destroy__();
        outputVertices.__destroy__();
        outputIndices .__destroy__();
        vertexMap     .__destroy__();
        return remeshedBufferGeo;
    }

    /** @returns {THREE.BufferGeometry} */
    simplifyMesh(vertices, indices, targetTriangleCount = 5000, maximalTriangleEdgeLength = 110.0){
        let inputVertices  = new this.px.PxArray_PxVec3(vertices.length/3);
        let inputIndices   = new this.px.PxArray_PxU32 (indices.length);
        for(let i = 0; i < vertices.length; i+=3){
            inputVertices.set(i/3, new this.px.PxVec3(vertices[i], vertices[i+1], vertices[i+2]));
        }
        for(let i = 0; i < indices.length; i++){
            inputIndices.set(i, indices[i]);
        }

        let outputVertices = new this.px.PxArray_PxVec3();
        let outputIndices  = new this.px.PxArray_PxU32 ();
        this.px.PxTetMaker.prototype.simplifyTriangleMesh(inputVertices, inputIndices, targetTriangleCount, maximalTriangleEdgeLength, outputVertices, outputIndices);

        console.log(inputVertices.size(), inputIndices.size(), outputVertices.size(), outputIndices.size());

        // Transform From PxVec3 to THREE.Vector3
        let triIndices = new Uint32Array(outputIndices.size());
        for(let i = 0; i < triIndices.length; i++){
            triIndices[i] = outputIndices.get(i);
        }
        let vertPositions = new Float32Array(outputVertices.size() * 3);
        for(let i = 0; i < outputVertices.size(); i++){
            let vec3 = outputVertices.get(i);
            vertPositions[i*3+0] = vec3.get_x();
            vertPositions[i*3+1] = vec3.get_y();
            vertPositions[i*3+2] = vec3.get_z();
        }
        let remeshedBufferGeo = new THREE.BufferGeometry();
        remeshedBufferGeo.setAttribute('position', new THREE.BufferAttribute(vertPositions, 3));
        remeshedBufferGeo.setIndex(new THREE.BufferAttribute(triIndices, 1));
        remeshedBufferGeo.computeVertexNormals();
        inputVertices .__destroy__();
        inputIndices  .__destroy__();
        outputVertices.__destroy__();
        outputIndices .__destroy__();
        return remeshedBufferGeo;
    }

    /** @returns {THREE.BufferGeometry} */
    createConformingTetrahedronMesh(vertices, indices, minTetVolume = 0.01){
        // First need to get the data into PhysX
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

        let analysis = this.px.PxTetMaker.prototype.validateTriangleMesh(simpleMesh);
        if (!analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eVALID) || analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eMESH_IS_INVALID)){
            console.log(  "eVALID",                                  analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eVALID),
                        "\neZERO_VOLUME",                            analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eZERO_VOLUME),
                        "\neOPEN_BOUNDARIES",                        analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eOPEN_BOUNDARIES),
                        "\neSELF_INTERSECTIONS",                     analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eSELF_INTERSECTIONS),
                        "\neINCONSISTENT_TRIANGLE_ORIENTATION",      analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eINCONSISTENT_TRIANGLE_ORIENTATION),
                        "\neCONTAINS_ACUTE_ANGLED_TRIANGLES",        analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eCONTAINS_ACUTE_ANGLED_TRIANGLES),
                        "\neEDGE_SHARED_BY_MORE_THAN_TWO_TRIANGLES", analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eEDGE_SHARED_BY_MORE_THAN_TWO_TRIANGLES),
                        "\neCONTAINS_DUPLICATE_POINTS",              analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eCONTAINS_DUPLICATE_POINTS),
                        "\neCONTAINS_INVALID_POINTS",                analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eCONTAINS_INVALID_POINTS),
                        "\neREQUIRES_32BIT_INDEX_BUFFER",            analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eREQUIRES_32BIT_INDEX_BUFFER),
                        "\neTRIANGLE_INDEX_OUT_OF_RANGE",            analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eTRIANGLE_INDEX_OUT_OF_RANGE),
                        "\neMESH_IS_PROBLEMATIC",                    analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eMESH_IS_PROBLEMATIC),
                        "\neMESH_IS_INVALID",                        analysis.isSet(this.px.PxTriangleMeshAnalysisResultEnum.eMESH_IS_INVALID));
        }
        
        // Now we should be able to make the Conforming Tetrahedron Mesh
        let outputVertices = new this.px.PxArray_PxVec3();
        let outputIndices  = new this.px.PxArray_PxU32 ();
        this.px.PxTetMaker.prototype.createConformingTetrahedronMesh(simpleMesh, outputVertices, outputIndices, true, minTetVolume);

        // Transform From PxVec3 to THREE.Vector3
        let tetIndices = new Uint32Array(outputIndices.size());
        for(let i = 0; i < tetIndices.length; i++){
            tetIndices[i] = outputIndices.get(i);
        }

        // Transform from Tet Indices to Edge Indices
        let segIndices = new Uint32Array((outputIndices.size()/4) * 12);
        for(let i = 0; i < outputIndices.size()/4; i++){
            let a = outputIndices.get(i * 4 + 0);
            let b = outputIndices.get(i * 4 + 1);
            let c = outputIndices.get(i * 4 + 2);
            let d = outputIndices.get(i * 4 + 3);
            segIndices[i*12+ 0]  = a;
            segIndices[i*12+ 1]  = b;
            segIndices[i*12+ 2]  = a;
            segIndices[i*12+ 3]  = c;
            segIndices[i*12+ 4]  = a;
            segIndices[i*12+ 5]  = d;
            segIndices[i*12+ 6]  = b;
            segIndices[i*12+ 7]  = c;
            segIndices[i*12+ 8]  = b;
            segIndices[i*12+ 9]  = d;
            segIndices[i*12+10] = c;
            segIndices[i*12+11] = d;
        }

        let vertPositions = new Float32Array(outputVertices.size() * 3);
        for(let i = 0; i < outputVertices.size(); i++){
            let vec3 = outputVertices.get(i);
            vertPositions[i*3+0] = vec3.get_x();
            vertPositions[i*3+1] = vec3.get_y();
            vertPositions[i*3+2] = vec3.get_z();
        }
        let remeshedBufferGeo = new THREE.BufferGeometry();
        remeshedBufferGeo.setAttribute('position', new THREE.BufferAttribute(vertPositions, 3));
        remeshedBufferGeo.setIndex(new THREE.BufferAttribute(segIndices, 1));

        inputVertices .__destroy__();
        inputIndices  .__destroy__();
        vertexData    .__destroy__();
        indexData     .__destroy__();
        simpleMesh    .__destroy__();
        outputVertices.__destroy__();
        outputIndices .__destroy__();

        return {
            numTets     : tetIndices.length / 4,
            numTetEdges : segIndices.length / 2,
            vertices    : vertPositions,
            tetIDs      : tetIndices,
            tetEdgeIDs  : segIndices,
            edgeMesh    : remeshedBufferGeo
        };
    }

    /** Calculate the barycentric coordinates of a point in a tetrahedron
     * https://stackoverflow.com/a/38546111
     * @param {THREE.Vector3} a
     * @param {THREE.Vector3} b
     * @param {THREE.Vector3} c
     * @param {THREE.Vector3} d
     * @param {THREE.Vector3} p
     * @returns {THREE.Vector4} */
    baryCoordTet(a, b, c, d, p){
        let vap = p.copy().sub(a);
        let vbp = p.copy().sub(b);
        let vab = b.copy().sub(a);
        let vac = c.copy().sub(a);
        let vad = d.copy().sub(a);
        let vbc = c.copy().sub(b);
        let vbd = d.copy().sub(b);
        // ScTP computes the scalar triple product
        let va6 = ScTP(vbp, vbd, vbc);
        let vb6 = ScTP(vap, vac, vad);
        let vc6 = ScTP(vap, vad, vab);
        let vd6 = ScTP(vap, vab, vac);
        let v6 = 1.0 / ScTP(vab, vac, vad);
        return new THREE.Vector4(va6*v6, vb6*v6, vc6*v6, vd6*v6);
    }

    // ScTP computes the scalar triple product
    ScTP(a, b, c){
        return a.dot(b.cross(c));
    }

    /** @param {THREE.Mesh} mesh */
    centerMesh(mesh){
        let bbox = new THREE.Box3().setFromObject(mesh);
        let magnitude = bbox.getSize(new THREE.Vector3()).length();
        mesh.scale.divideScalar(magnitude / 2.5);
        bbox = new THREE.Box3().setFromObject(mesh);
        let center = new THREE.Vector3();
        bbox.getCenter(center);
        mesh.position.sub(center);
        bbox = new THREE.Box3().setFromObject(mesh);
        mesh.position.y -= bbox.min.y;
    }
}

var main = new Main();
