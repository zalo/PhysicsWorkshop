// Web Worker: cooks triangle meshes with SDF off the main thread
// Receives: { id, vertices: Float32Array, indices: Uint32Array, spacing: number }
// Returns:  { id, cookedData: ArrayBuffer } or { id, error: string }

let px = null;
let cookingParams = null;

async function init() {
    const PhysXInit = (await import('../assets/physx-js-webidl/dist/physx-js-webidl.mjs')).default;
    px = await PhysXInit();

    let version = px.PHYSICS_VERSION;
    let allocator = new px.PxDefaultAllocator();
    let errorCb = new px.PxDefaultErrorCallback();
    let foundation = px.CreateFoundation(version, allocator, errorCb);
    let tolerances = new px.PxTolerancesScale();
    px.CreatePhysics(version, foundation, tolerances);

    cookingParams = new px.PxCookingParams(tolerances);
    cookingParams.get_meshPreprocessParams().raise(px.eENABLE_INERTIA);

    postMessage({ type: 'ready' });
}

onmessage = async function(e) {
    if (!px) await init();

    let { id, vertices, indices, spacing } = e.data;
    try {
        let numVerts = vertices.length / 3;
        let numTris = indices.length / 3;

        let inputVerts = new px.PxArray_PxVec3(numVerts);
        for (let i = 0; i < numVerts; i++) {
            inputVerts.set(i, new px.PxVec3(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]));
        }
        let inputTris = new px.PxArray_PxU32(indices.length);
        for (let i = 0; i < indices.length; i++) {
            inputTris.set(i, indices[i]);
        }

        let pointsData = new px.PxBoundedData();
        pointsData.set_count(numVerts);
        pointsData.set_stride(12);
        pointsData.set_data(inputVerts.begin());

        let trisData = new px.PxBoundedData();
        trisData.set_count(numTris);
        trisData.set_stride(12);
        trisData.set_data(inputTris.begin());

        let desc = new px.PxTriangleMeshDesc();
        desc.set_points(pointsData);
        desc.set_triangles(trisData);

        let sdfDesc = new px.PxSDFDesc();
        sdfDesc.set_spacing(spacing);
        sdfDesc.set_subgridSize(0);
        sdfDesc.set_bitsPerSubgridPixel(px.e32_BIT_PER_PIXEL);
        sdfDesc.set_numThreadsForSdfConstruction(1);
        desc.set_sdfDesc(sdfDesc);

        // Cook to output stream (serialized bytes)
        let outStream = new px.PxDefaultMemoryOutputStream();
        let t0 = performance.now();
        let ok = px.CookTriangleMesh(cookingParams, desc, outStream);
        let dt = performance.now() - t0;

        inputVerts.__destroy__();
        inputTris.__destroy__();
        pointsData.__destroy__();
        trisData.__destroy__();
        desc.__destroy__();
        sdfDesc.__destroy__();

        if (!ok) {
            outStream.__destroy__();
            postMessage({ id, error: 'CookTriangleMesh failed' });
            return;
        }

        // Copy cooked data out of WASM heap byte by byte (safe against memory growth)
        let size = outStream.getSize();
        let ptr = outStream.getData();
        let cookedData = new ArrayBuffer(size);
        let dst = new Uint8Array(cookedData);
        for (let i = 0; i < size; i++) {
            dst[i] = px.HEAPU8[ptr + i];
        }
        outStream.__destroy__();

        postMessage({ id, cookedData, bakeTimeMs: dt }, [cookedData]);
    } catch (e) {
        postMessage({ id, error: e.message });
    }
};
