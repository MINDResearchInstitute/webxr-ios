
import MetalKit
import ARKit

protocol RenderDestinationProvider {
    var currentRenderPassDescriptor: MTLRenderPassDescriptor? { get }
    var currentDrawable: CAMetalDrawable? { get }
    var colorPixelFormat: MTLPixelFormat { get set }
    var depthStencilPixelFormat: MTLPixelFormat { get set }
    var sampleCount: Int { get set }
}

let maxBuffersInFlight: Int = 3
let maxAnchorInstanceCount: Int = 64

//Per-Cell Data Fields
let valuesPerCell = 8

let TOTAL_WEIGHT = 0
let TYPE_FLAGS = 1
let TYPE_AVERAGE = 2
let X_COORD_AVERAGE = 3
let Y_COORD_AVERAGE = 4
let X_ORIENTATION_AVERAGE = 5
let Y_ORIENTATION_AVERAGE = 6
let DOT_SIZE = 7
let sizeInt32 = MemoryLayout<Int32>.stride
let NUM_PIX_X = 1920
let NUM_PIX_Y = 1080
let BYTES_PER_ROW = 7680
let GRID_RESOLUTION = 1
var gridDivisionsX = 16*GRID_RESOLUTION
var gridDivisionsY = 9*GRID_RESOLUTION
var clinkDataSize = valuesPerCell*gridDivisionsX*gridDivisionsY + numTagTypes
var numGridCells = gridDivisionsX*gridDivisionsY

//Debug lines. We can remove later:
var hLine:Int32 = 1920/2
var vLine:Int32 = 400

//Corner Types
let numTagTypes = 30
let BOARD_3Part_CW = 0
let CODE_3Part_CW = 1
let BOARD_3Part_CCW = 2
let CODE_3Part_CCW = 3
let BOARD_4Part_RR = 4
let MARKER_4Part_YY = 5
let BOARD_4Part_BB = 6

let VALID_CLINKCODE_DIAGONALS = [28, 23, 49, 19, 52, 46, 13, 59]


struct ExtractedTag{
    var weight:Double
    var typeFlags:Int32
    var type:Int
    var x:Double
    var y:Double
    var orientX:Double
    var orientY:Double
    var orientHypot:Double
    var dotSize:Double
    var cellIndex:Int
    var xy: (x:Double, y:Double) { return (self.x, self.y) }
    
    func distTo(_ tag:ExtractedTag ) -> Double{
        return hypot(tag.x-x, tag.y-y)
    }
    
    func getErrorPointingTo(_ tag:ExtractedTag ) -> Double{
        let dist = self.distTo(tag)
        let pxy = (x:self.x + dist*self.orientX/self.orientHypot, y:self.y + dist*self.orientY/self.orientHypot)
        let error = hypot(pxy.x - tag.x, pxy.y - tag.y)
        return error
    }
}

struct OppositeTagPair{
    var tagA:ExtractedTag
    var tagB:ExtractedTag
    var centerXY:(x:Double, y:Double) {
        return (x:(self.tagA.x + self.tagB.x)/2.0, y:(self.tagA.y + self.tagB.y)/2.0)
    }
    var separation:Double {
        return hypot(self.tagA.x - self.tagB.x, self.tagA.y - self.tagB.y)
    }
    
    func isCompatable(_ withPair:OppositeTagPair ) -> Bool{
        let dist:Double = hypot( self.centerXY.x - withPair.centerXY.x, self.centerXY.y - withPair.centerXY.y )
        let distThresh:Double = min(self.separation, withPair.separation) / 2.0
        return dist < distThresh
    }
}

struct ClinkCode{
    var topLeft:ExtractedTag
    var topRight:ExtractedTag
    var bottomLeft:ExtractedTag
    var bottomRight:ExtractedTag
    
    var isValid:Bool = false
    var projectionMatrix:[Double] = []
    var code:Int32 = 0
    var centerXY:(x:Double, y:Double) = (-1,-1)
    
    init( cwPair:OppositeTagPair, ccwPair:OppositeTagPair, pixelBufferBaseAddress:UnsafeRawPointer ){
        (topLeft, bottomRight) = (cwPair.tagA, cwPair.tagB)
        if(topLeft.y > bottomRight.y){
            (topLeft, bottomRight) = (bottomRight, topLeft) //swap assuming up is actually up
        }
        (topRight, bottomLeft) = (ccwPair.tagA, ccwPair.tagB)
        let errorToBottom = topLeft.getErrorPointingTo(bottomLeft)
        let errorToRight = topLeft.getErrorPointingTo(topRight)
        if(errorToRight < errorToBottom){
            (bottomLeft, topRight) = (topRight, bottomLeft) //swap to align better
        }
        projectionMatrix = buildClinkcodeProjectionMatrix(&self)
        guard !projectionMatrix.contains(where:{!$0.isFinite}) else{
            return
        }
        code = self.readCode(pixelBufferBaseAddress)
        guard code != 0 else{
            return
        }
        if(code == -1){
            //rotate by 180 degrees
            (topLeft, topRight, bottomRight, bottomLeft ) = ( bottomRight, bottomLeft, topLeft, topRight )
            projectionMatrix = buildClinkcodeProjectionMatrix(&self)
            code = self.readCode(pixelBufferBaseAddress)
            guard code > 0 else{
                return
            }
        }
        centerXY = projectPoint(m:projectionMatrix, pt:(x:2.5, y:2.5))
        isValid = true
    }
    
    func readCode(_ pixelBufferBaseAddress:UnsafeRawPointer ) -> Int32 {
        var diagonalA:[Int] = []
        var diagonalB:[Int] = []
        let m = self.projectionMatrix
        var avgLum:Int = 0
        for i in 0..<6 {
            var pt = projectPoint(m:m , pt: (x:Double(i), y: Double(i)))
            var rgb = readRBGFromPixelBuffer(baseAddress: pixelBufferBaseAddress, xy:pt)
            guard rgb != nil else{
                return 0
            }
            var lum = Int(rgb!.r) + Int(rgb!.g) + Int(rgb!.b)
            avgLum += lum
            diagonalA.append(lum)
            pt = projectPoint(m:m , pt: (x:Double(5-i), y: Double(i)))
            rgb = readRBGFromPixelBuffer(baseAddress: pixelBufferBaseAddress, xy:pt)
            guard rgb != nil else{
                return 0
            }
            lum = Int(rgb!.r) + Int(rgb!.g) + Int(rgb!.b)
            avgLum += lum
            diagonalB.append(lum)
        }
        avgLum = avgLum / 12
        var diagonalValue:Int = 0
        var reverseDiagonalValue:Int = 0
        for i in 0..<6 {
            diagonalA[i] = (diagonalA[i] < avgLum) ? 1 : 0
            diagonalB[i] = (diagonalB[i] < avgLum) ? 1 : 0
            guard diagonalA[i] != diagonalB[i] else{
                return 0 //0 here means no chance of it working after a 180deg rotation
            }
            if(diagonalA[i] == 1){
                reverseDiagonalValue = reverseDiagonalValue | (1 << i)
                diagonalValue = diagonalValue | (1 << (5-i))
            }
        }
        
        if( VALID_CLINKCODE_DIAGONALS.contains(reverseDiagonalValue )){
            return -1 // here -1 means it'll work if we do a 180deg rotation
        }
        guard VALID_CLINKCODE_DIAGONALS.contains(diagonalValue) else{
            return 0 // no chance to work after 180deg rot
        }
        
        var codeValue:Int32 = 0
        var bitIndex:Int = 0
        for x in 0..<6 {
            for y in 0..<6 {
                guard x != y && x != 5-y else{
                    continue
                }
                let pt = projectPoint(m:m , pt: (x:Double(x), y: Double(y)))
                let rgb = readRBGFromPixelBuffer(baseAddress: pixelBufferBaseAddress, xy:pt)
                guard rgb != nil else{
                    return 0
                }
                let lum = Int(rgb!.r) + Int(rgb!.g) + Int(rgb!.b)
                if(lum < avgLum){
                    codeValue = codeValue | (1 << bitIndex)
                }
                bitIndex = bitIndex + 1
            }
        }
        let binString = String(codeValue, radix:2)
        let hash = (simpleHash(binString) - 12) % 64
        guard hash == diagonalValue else{
            return 0
        }
        return codeValue
    }
}

func isClinkboardType(type: Int) -> Bool {
    switch type {
    case BOARD_3Part_CW, BOARD_3Part_CCW, BOARD_4Part_BB, BOARD_4Part_RR :
        return true
    default:
        return false
    }
}

func isClinkcodeType(type: Int) -> Bool {
    switch type {
    case CODE_3Part_CW, CODE_3Part_CCW :
        return true
    default:
        return false
    }
}

func readRBGFromPixelBuffer( baseAddress:UnsafeRawPointer, xy:(x:Double, y:Double) ) -> (r:UInt8, g:UInt8, b:UInt8)? {
    let x = Int(round(xy.x))
    let y = Int(round(xy.y))
    guard x >= 0 && x < NUM_PIX_X && y >= 0 && y < NUM_PIX_Y else{
        return nil
    }
    let offset:Int = y*BYTES_PER_ROW + x*4
    let r:UInt8 = baseAddress.load(fromByteOffset: offset+0, as: UInt8.self)
    let g:UInt8 = baseAddress.load(fromByteOffset: offset+1, as: UInt8.self)
    let b:UInt8 = baseAddress.load(fromByteOffset: offset+2, as: UInt8.self)
    return (r:r,g:g,b:b)
}

func simpleHash(_ s:String ) -> Int {
    var h = 0;
    for c in s.utf8{
        h = (31*h + Int(c)) & 0xffffffff;
    }
    return h
}

@objc class MINDMetalRenderer:NSObject {
    
    var session: ARSession!
    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!
    let inFlightSemaphore = DispatchSemaphore(value: maxBuffersInFlight)
    var renderDestination: MTKView!
    var sharedUniformBuffer: MTLBuffer!
    var anchorUniformBuffer: MTLBuffer!
    var debugUniformBuffer: MTLBuffer!
    var imagePlaneVertexBuffer: MTLBuffer!
    var convertToRGBPipelineState: MTLRenderPipelineState!
    var capturedImagePipelineState: MTLRenderPipelineState!
    var capturedImageDepthState: MTLDepthStencilState!
    var anchorPipelineState: MTLRenderPipelineState!
    var anchorDepthState: MTLDepthStencilState!
    var debugPipelineState: MTLRenderPipelineState!
    var debugDepthState: MTLDepthStencilState!
    var capturedImageTextureY: MTLTexture!
    var capturedImageTextureCbCr: MTLTexture!
    var convertedImageTextureRGB: MTLTexture!
    var conversionPassDescriptor: MTLRenderPassDescriptor!
    var capturedImageTextureCache: CVMetalTextureCache!
    var geometryVertexDescriptor: MTLVertexDescriptor!
    var mesh: MTKMesh!
    var debugMesh: MTKMesh!
    var uniformBufferIndex: Int = 0
    var sharedUniformBufferOffset: Int = 0
    var anchorUniformBufferOffset: Int = 0
    var debugUniformBufferOffset: Int = 0
    var sharedUniformBufferAddress: UnsafeMutableRawPointer!
    var anchorUniformBufferAddress: UnsafeMutableRawPointer!
    var debugUniformBufferAddress: UnsafeMutableRawPointer!
    var debugInstanceCount: Int = 0
    var anchorInstanceCount: Int = 0
    var viewportSize: CGSize = CGSize()
    var viewportSizeDidChange: Bool = false
    
    struct SharedUniforms {
        var projectionMatrix: matrix_float4x4
        var viewMatrix: matrix_float4x4
        var ambientLightColor: vector_float3
        var directionalLightDirection: vector_float3
        var directionalLightColor: vector_float3
        var materialShininess: Float
    }
    
    struct InstanceUniforms {
        var modelMatrix: matrix_float4x4
    }
    
    let alignedSharedUniformSize = (MemoryLayout<SharedUniforms>.size & ~0xFF) + 0x100
    let alignedInstanceUniformSize = ((MemoryLayout<InstanceUniforms>.size * maxAnchorInstanceCount) & ~0xFF) + 0x100
    let planeVertexData: [Float] = [-1, -1,  0,  1,
                                    1, -1,  1,  1,
                                    -1,  1,  0,  0,
                                    1,  1,  1,  0]
    
    //Clink Data
    internal var clinkData = [Int32](repeating:0, count: clinkDataSize)
    internal var clinkDataBuffer: MTLBuffer?
    
//    override public init() {
//        super.init()
//    }
//    
//    public override init() {
//        super.init()
//    }
    
    
    @objc func setup(session: ARSession, device: MTLDevice, view: MTKView) {
        self.session = session
        self.device = device
        self.renderDestination = view
        setupPipeline()
        setupAssets()
    }
    
    func setupPipeline() {
        
        //Intermediate Texture to store RGB texture and pass into clink shader
        let textureDescriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm,
                                                                         width: NUM_PIX_X,
                                                                         height: NUM_PIX_Y,
                                                                         mipmapped: false)
        textureDescriptor.usage = [.shaderRead, .shaderWrite, .renderTarget]
        convertedImageTextureRGB = device.makeTexture(descriptor: textureDescriptor)!
        
        //let rpd = renderDestination.currentRenderPassDescriptor
        
        conversionPassDescriptor = MTLRenderPassDescriptor()
        conversionPassDescriptor.colorAttachments[0].clearColor = MTLClearColorMake(1, 1, 1, 1)
        conversionPassDescriptor.colorAttachments[0].loadAction = .load
        conversionPassDescriptor.colorAttachments[0].storeAction = .store
        conversionPassDescriptor.colorAttachments[0].texture = convertedImageTextureRGB
        
        renderDestination.depthStencilPixelFormat = .depth32Float_stencil8
        renderDestination.colorPixelFormat = .bgra8Unorm
        renderDestination.sampleCount = 1
        let sharedUniformBufferSize = alignedSharedUniformSize * maxBuffersInFlight
        let anchorUniformBufferSize = alignedInstanceUniformSize * maxBuffersInFlight
        sharedUniformBuffer = device.makeBuffer(length: sharedUniformBufferSize, options: .storageModeShared)
        anchorUniformBuffer = device.makeBuffer(length: anchorUniformBufferSize, options: .storageModeShared)
        debugUniformBuffer = device.makeBuffer(length: anchorUniformBufferSize, options: .storageModeShared)
        let imagePlaneVertexDataCount = planeVertexData.count * MemoryLayout<Float>.size
        imagePlaneVertexBuffer = device.makeBuffer(bytes: planeVertexData, length: imagePlaneVertexDataCount, options: [])
        let defaultLibrary = device.makeDefaultLibrary()!
        
        let capturedImageVertexFunction = defaultLibrary.makeFunction(name: "capturedImageVertexTransform")!
        let imagePlaneVertexDescriptor = MTLVertexDescriptor()
        imagePlaneVertexDescriptor.attributes[0].format = .float2
        imagePlaneVertexDescriptor.attributes[0].offset = 0
        imagePlaneVertexDescriptor.attributes[0].bufferIndex = 0
        imagePlaneVertexDescriptor.attributes[1].format = .float2
        imagePlaneVertexDescriptor.attributes[1].offset = 8
        imagePlaneVertexDescriptor.attributes[1].bufferIndex = 0
        imagePlaneVertexDescriptor.layouts[0].stride = 16
        imagePlaneVertexDescriptor.layouts[0].stepRate = 1
        imagePlaneVertexDescriptor.layouts[0].stepFunction = .perVertex
        
        let convertToRGBFragmentFunction = defaultLibrary.makeFunction(name: "ycbcrToRGBFragmentShader")!
        
        let convertToRGBPipelineStateDescriptor = MTLRenderPipelineDescriptor()
        convertToRGBPipelineStateDescriptor.label = "ConvertToRGBPipeline"
        convertToRGBPipelineStateDescriptor.vertexFunction = capturedImageVertexFunction
        convertToRGBPipelineStateDescriptor.fragmentFunction = convertToRGBFragmentFunction
        convertToRGBPipelineStateDescriptor.vertexDescriptor = imagePlaneVertexDescriptor
        convertToRGBPipelineStateDescriptor.sampleCount = 1
        convertToRGBPipelineStateDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
        convertToRGBPipelineStateDescriptor.depthAttachmentPixelFormat = .invalid
        do { try convertToRGBPipelineState = device.makeRenderPipelineState(descriptor: convertToRGBPipelineStateDescriptor) }
        catch let error { print("Failed to created convertToRGB pipeline state, error \(error)") }
        
        
        let pipelineDescriptor = MTLRenderPipelineDescriptor()
        pipelineDescriptor.sampleCount = 1
        pipelineDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
        pipelineDescriptor.depthAttachmentPixelFormat = .invalid
        
        
        let clinkVertexFunction = defaultLibrary.makeFunction(name: "mapTexture")
        
        let capturedImageFragmentFunction = defaultLibrary.makeFunction(name: "clinkFragmentShader")!
        let capturedImagePipelineStateDescriptor = MTLRenderPipelineDescriptor()
        capturedImagePipelineStateDescriptor.label = "MyCapturedImagePipeline"
        capturedImagePipelineStateDescriptor.sampleCount = renderDestination.sampleCount
        capturedImagePipelineStateDescriptor.vertexFunction = clinkVertexFunction
        capturedImagePipelineStateDescriptor.fragmentFunction = capturedImageFragmentFunction
        capturedImagePipelineStateDescriptor.vertexDescriptor = imagePlaneVertexDescriptor
        capturedImagePipelineStateDescriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        capturedImagePipelineStateDescriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        capturedImagePipelineStateDescriptor.stencilAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        do { try capturedImagePipelineState = device.makeRenderPipelineState(descriptor: capturedImagePipelineStateDescriptor) }
        catch let error { print("Failed to created captured image pipeline state, error \(error)") }
        let capturedImageDepthStateDescriptor = MTLDepthStencilDescriptor()
        capturedImageDepthStateDescriptor.depthCompareFunction = .always
        capturedImageDepthStateDescriptor.isDepthWriteEnabled = false
        capturedImageDepthState = device.makeDepthStencilState(descriptor: capturedImageDepthStateDescriptor)
        var textureCache: CVMetalTextureCache?
        CVMetalTextureCacheCreate(nil, nil, device, nil, &textureCache)
        capturedImageTextureCache = textureCache
        let anchorGeometryVertexFunction = defaultLibrary.makeFunction(name: "anchorGeometryVertexTransform")!
        let anchorGeometryFragmentFunction = defaultLibrary.makeFunction(name: "anchorGeometryFragmentLighting")!
        geometryVertexDescriptor = MTLVertexDescriptor()
        geometryVertexDescriptor.attributes[0].format = .float3
        geometryVertexDescriptor.attributes[0].offset = 0
        geometryVertexDescriptor.attributes[0].bufferIndex = 0
        geometryVertexDescriptor.attributes[1].format = .float2
        geometryVertexDescriptor.attributes[1].offset = 0
        geometryVertexDescriptor.attributes[1].bufferIndex = 1
        geometryVertexDescriptor.attributes[2].format = .half3
        geometryVertexDescriptor.attributes[2].offset = 8
        geometryVertexDescriptor.attributes[2].bufferIndex = 1
        geometryVertexDescriptor.layouts[0].stride = 12
        geometryVertexDescriptor.layouts[0].stepRate = 1
        geometryVertexDescriptor.layouts[0].stepFunction = .perVertex
        geometryVertexDescriptor.layouts[1].stride = 16
        geometryVertexDescriptor.layouts[1].stepRate = 1
        geometryVertexDescriptor.layouts[1].stepFunction = .perVertex
        let anchorPipelineStateDescriptor = MTLRenderPipelineDescriptor()
        anchorPipelineStateDescriptor.label = "MyAnchorPipeline"
        anchorPipelineStateDescriptor.sampleCount = renderDestination.sampleCount
        anchorPipelineStateDescriptor.vertexFunction = anchorGeometryVertexFunction
        anchorPipelineStateDescriptor.fragmentFunction = anchorGeometryFragmentFunction
        anchorPipelineStateDescriptor.vertexDescriptor = geometryVertexDescriptor
        anchorPipelineStateDescriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        anchorPipelineStateDescriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        anchorPipelineStateDescriptor.stencilAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        do { try anchorPipelineState = device.makeRenderPipelineState(descriptor: anchorPipelineStateDescriptor) }
        catch let error { print("Failed to created anchor geometry pipeline state, error \(error)") }
        let anchorDepthStateDescriptor = MTLDepthStencilDescriptor()
        anchorDepthStateDescriptor.depthCompareFunction = .less
        anchorDepthStateDescriptor.isDepthWriteEnabled = true
        anchorDepthState = device.makeDepthStencilState(descriptor: anchorDepthStateDescriptor)
        let debugGeometryVertexFunction = defaultLibrary.makeFunction(name: "vertexDebugPlane")!
        let debugGeometryFragmentFunction = defaultLibrary.makeFunction(name: "fragmentDebugPlane")!
        anchorPipelineStateDescriptor.vertexFunction =  debugGeometryVertexFunction
        anchorPipelineStateDescriptor.fragmentFunction = debugGeometryFragmentFunction
        do { try debugPipelineState = device.makeRenderPipelineState(descriptor: anchorPipelineStateDescriptor)
        } catch let error { print(error) }
        debugDepthState = device.makeDepthStencilState(descriptor: anchorDepthStateDescriptor)
        commandQueue = device.makeCommandQueue()
    }
    
    func setupAssets() {
        let metalAllocator = MTKMeshBufferAllocator(device: device)
        let vertexDescriptor = MTKModelIOVertexDescriptorFromMetal(geometryVertexDescriptor)
        (vertexDescriptor.attributes[0] as! MDLVertexAttribute).name = MDLVertexAttributePosition
        (vertexDescriptor.attributes[1] as! MDLVertexAttribute).name = MDLVertexAttributeTextureCoordinate
        (vertexDescriptor.attributes[2] as! MDLVertexAttribute).name = MDLVertexAttributeNormal
        var mdlMesh = MDLMesh(boxWithExtent: vector3(0.075, 0.075, 0.075), segments: vector3(1, 1, 1), inwardNormals: false, geometryType: .triangles, allocator: metalAllocator)
        mdlMesh.vertexDescriptor = vertexDescriptor
        do { try mesh = MTKMesh(mesh: mdlMesh, device: device) }
        catch let error { print("Error creating MetalKit mesh, error \(error)") }
        mdlMesh = MDLMesh(planeWithExtent: vector3(0.1, 0.1, 0.1), segments: vector2(1, 1), geometryType: .triangles, allocator: metalAllocator)
        mdlMesh.vertexDescriptor = vertexDescriptor
        do { try debugMesh = MTKMesh(mesh: mdlMesh, device: device)
        } catch let error { print(error) }
    }
    
    @objc func drawRectResized(size: CGSize) {
        viewportSize = size
        viewportSizeDidChange = true
    }
    
    @objc func update() {
        let _ = inFlightSemaphore.wait(timeout: DispatchTime.distantFuture)
        guard let commandBuffer = commandQueue.makeCommandBuffer() else { return }
        commandBuffer.addCompletedHandler{ [weak self] commandBuffer in
            if let strongSelf = self { strongSelf.inFlightSemaphore.signal() }
            return
        }
        updateBufferStates()
        updateGameState()
        
        clinkDataBuffer = device.makeBuffer(bytes: &clinkData,
                                            length: clinkDataSize * sizeInt32,
                                            options: .storageModeShared)
        
        guard let conversionRenderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: conversionPassDescriptor) else { return }
        convertTextureToRGB(renderEncoder: conversionRenderEncoder)
        conversionRenderEncoder.endEncoding()
        
        guard let renderPassDescriptor = renderDestination.currentRenderPassDescriptor,
            let drawable = renderDestination.currentDrawable else { return }
        guard let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else { return }
        detectAndRenderClinkData(renderEncoder: renderEncoder)
        //drawAnchorGeometry(renderEncoder: renderEncoder)
        //drawDebugGeometry(renderEncoder: renderEncoder)
        renderEncoder.endEncoding()
        commandBuffer.present(drawable)
        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()
        
        //processClinkData();
    }
    
    func updateBufferStates() {
        uniformBufferIndex = (uniformBufferIndex + 1) % maxBuffersInFlight
        sharedUniformBufferOffset = alignedSharedUniformSize * uniformBufferIndex
        anchorUniformBufferOffset = alignedInstanceUniformSize * uniformBufferIndex
        sharedUniformBufferAddress = sharedUniformBuffer.contents().advanced(by: sharedUniformBufferOffset)
        anchorUniformBufferAddress = anchorUniformBuffer.contents().advanced(by: anchorUniformBufferOffset)
        debugUniformBufferOffset = alignedInstanceUniformSize * uniformBufferIndex
        debugUniformBufferAddress = debugUniformBuffer.contents().advanced(by: debugUniformBufferOffset)
    }
    
    func updateGameState() {
        guard let currentFrame = session.currentFrame else { return }
        updateSharedUniforms(frame: currentFrame)
        updateAnchors(frame: currentFrame)
        updateCapturedImageTextures(frame: currentFrame)
        if viewportSizeDidChange {
            viewportSizeDidChange = false
            updateImagePlane(frame: currentFrame)
        }
    }
    
    func updateSharedUniforms(frame: ARFrame) {
        let uniforms = sharedUniformBufferAddress.assumingMemoryBound(to: SharedUniforms.self)
        uniforms.pointee.viewMatrix = simd_inverse(frame.camera.transform)
        uniforms.pointee.projectionMatrix = frame.camera.projectionMatrix(for: .portrait, viewportSize: viewportSize, zNear: 0.001, zFar: 1000)
        var ambientIntensity: Float = 1.0
        if let lightEstimate = frame.lightEstimate {
            ambientIntensity = Float(lightEstimate.ambientIntensity) / 1000.0
        }
        let ambientLightColor: vector_float3 = vector3(0.5, 0.5, 0.5)
        uniforms.pointee.ambientLightColor = ambientLightColor * ambientIntensity
        var directionalLightDirection : vector_float3 = vector3(0.0, 0.0, -1.0)
        directionalLightDirection = simd_normalize(directionalLightDirection)
        uniforms.pointee.directionalLightDirection = directionalLightDirection
        let directionalLightColor: vector_float3 = vector3(0.6, 0.6, 0.6)
        uniforms.pointee.directionalLightColor = directionalLightColor * ambientIntensity
        uniforms.pointee.materialShininess = 30
    }
    
    func updateAnchors(frame: ARFrame) {
        anchorInstanceCount = min(frame.anchors.count, maxAnchorInstanceCount)
        var anchorOffset: Int = 0
        if anchorInstanceCount == maxAnchorInstanceCount {
            anchorOffset = max(frame.anchors.count - maxAnchorInstanceCount, 0)
        }
        let count = frame.anchors.filter{ $0.isKind(of: ARPlaneAnchor.self) }.count
        debugInstanceCount = min(count, maxAnchorInstanceCount - (anchorInstanceCount - count))
        for index in 0..<anchorInstanceCount {
            let anchor = frame.anchors[index + anchorOffset]
            var coordinateSpaceTransform = matrix_identity_float4x4
            coordinateSpaceTransform.columns.2.z = -1.0 // flip Z axis to convert to left handed
            if anchor.isKind(of: ARPlaneAnchor.self) {
                let transform = anchor.transform * rotationMatrix(rotation: float3(0, 0, Float.pi/2))
                let modelMatrix = simd_mul(transform, coordinateSpaceTransform)
                let debugUniforms = debugUniformBufferAddress.assumingMemoryBound(to: InstanceUniforms.self).advanced(by: index)
                debugUniforms.pointee.modelMatrix = modelMatrix
            } else {
                let modelMatrix = simd_mul(anchor.transform, coordinateSpaceTransform)
                let anchorUniforms = anchorUniformBufferAddress.assumingMemoryBound(to: InstanceUniforms.self).advanced(by: index)
                anchorUniforms.pointee.modelMatrix = modelMatrix
            }
        }
    }
    
    func rotationMatrix(rotation: float3) -> float4x4 {
        var matrix: float4x4 = matrix_identity_float4x4
        let x = rotation.x
        let y = rotation.y
        let z = rotation.z
        matrix.columns.0.x = cos(y) * cos(z)
        matrix.columns.0.y = cos(z) * sin(x) * sin(y) - cos(x) * sin(z)
        matrix.columns.0.z = cos(x) * cos(z) * sin(y) + sin(x) * sin(z)
        matrix.columns.1.x = cos(y) * sin(z)
        matrix.columns.1.y = cos(x) * cos(z) + sin(x) * sin(y) * sin(z)
        matrix.columns.1.z = -cos(z) * sin(x) + cos(x) * sin(y) * sin(z)
        matrix.columns.2.x = -sin(y)
        matrix.columns.2.y = cos(y) * sin(x)
        matrix.columns.2.z = cos(x) * cos(y)
        matrix.columns.3.w = 1.0
        return matrix
    }
    
    func updateCapturedImageTextures(frame: ARFrame) {
        let pixelBuffer = frame.capturedImage
        if (CVPixelBufferGetPlaneCount(pixelBuffer) < 2) { return }
        capturedImageTextureY = createTexture(fromPixelBuffer: pixelBuffer, pixelFormat:.r8Unorm, planeIndex:0)!
        capturedImageTextureCbCr = createTexture(fromPixelBuffer: pixelBuffer, pixelFormat:.rg8Unorm, planeIndex:1)!
    }
    
    func createTexture(fromPixelBuffer pixelBuffer: CVPixelBuffer, pixelFormat: MTLPixelFormat, planeIndex: Int) -> MTLTexture? {
        var mtlTexture: MTLTexture? = nil
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, planeIndex)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, planeIndex)
        var texture: CVMetalTexture? = nil
        let status = CVMetalTextureCacheCreateTextureFromImage(nil, capturedImageTextureCache, pixelBuffer, nil, pixelFormat, width, height, planeIndex, &texture)
        if status == kCVReturnSuccess { mtlTexture = CVMetalTextureGetTexture(texture!) }
        return mtlTexture
    }
    
    func updateImagePlane(frame: ARFrame) {
        let displayToCameraTransform = frame.displayTransform(for: .portrait, viewportSize: viewportSize).inverted()
        let vertexData = imagePlaneVertexBuffer.contents().assumingMemoryBound(to: Float.self)
        for index in 0...3 {
            let textureCoordIndex = 4 * index + 2
            let textureCoord = CGPoint(x: CGFloat(planeVertexData[textureCoordIndex]), y: CGFloat(planeVertexData[textureCoordIndex + 1]))
            let transformedCoord = textureCoord.applying(displayToCameraTransform)
            vertexData[textureCoordIndex] = Float(transformedCoord.x)
            vertexData[textureCoordIndex + 1] = Float(transformedCoord.y)
        }
    }
    
    
    
    
    func convertTextureToRGB(renderEncoder: MTLRenderCommandEncoder) {
        guard capturedImageTextureY != nil && capturedImageTextureCbCr != nil && convertedImageTextureRGB != nil else { return }
        renderEncoder.pushDebugGroup("convertTextureToRGB")
        renderEncoder.setCullMode(.none)
        renderEncoder.setRenderPipelineState(convertToRGBPipelineState)
        renderEncoder.setFragmentTexture(capturedImageTextureY, index: 1)
        renderEncoder.setFragmentTexture(capturedImageTextureCbCr, index: 2)
        renderEncoder.setDepthStencilState(capturedImageDepthState)
        renderEncoder.setVertexBuffer(imagePlaneVertexBuffer, offset: 0, index: 0)
        renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
        renderEncoder.popDebugGroup()
    }
    
    func detectAndRenderClinkData(renderEncoder: MTLRenderCommandEncoder ) {
        
        
        guard capturedImageTextureY != nil && capturedImageTextureCbCr != nil else { return }
        renderEncoder.pushDebugGroup("detectAndRenderClinkData")
        renderEncoder.setCullMode(.none)
        renderEncoder.setRenderPipelineState(capturedImagePipelineState)
        renderEncoder.setDepthStencilState(capturedImageDepthState)
        renderEncoder.setVertexBuffer(imagePlaneVertexBuffer, offset: 0, index: 0)
        renderEncoder.setFragmentTexture(convertedImageTextureRGB, index: 0)
        renderEncoder.setFragmentBuffer(clinkDataBuffer, offset: 0, index: 0)
        renderEncoder.setFragmentBytes(&hLine, length: sizeInt32, index: 1)
        renderEncoder.setFragmentBytes(&vLine, length: sizeInt32, index: 2)
        renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
        renderEncoder.popDebugGroup()
        
    }
    
    func processClinkData(){
        //let t0 = CFAbsoluteTimeGetCurrent()
        hLine = Int32(0)
        vLine = Int32(0)
        
        let data = (clinkDataBuffer?.contents().bindMemory(to: Int32.self, capacity: clinkDataSize))!
        
        var clinkboardDetected = (data[BOARD_4Part_RR] > 0 && data[BOARD_4Part_BB] > 0 && data[BOARD_3Part_CW] > 0 && data[BOARD_3Part_CCW] > 0)
        var clinkcodeDetected = (data[CODE_3Part_CW] > 1 && data[CODE_3Part_CCW] > 1)
        
        if(clinkboardDetected || clinkcodeDetected) {
            
            let bytesPerRow = 4*NUM_PIX_X
            let length = NUM_PIX_Y*bytesPerRow
            let rgbaBytes = [UInt8](repeating: 0, count: length)
            let pixelBufferPointer = UnsafeMutableRawPointer(mutating: rgbaBytes)
            let region = MTLRegionMake2D(0, 0, NUM_PIX_X, NUM_PIX_Y)
            convertedImageTextureRGB.getBytes(pixelBufferPointer, bytesPerRow: bytesPerRow, from: region, mipmapLevel: 0)
            
//            let imageBuffer = CMSampleBufferGetImageBuffer(self.sampleBuffer!)
//            CVPixelBufferLockBaseAddress(imageBuffer!,[])
//            let baseAddress = CVPixelBufferGetBaseAddress(imageBuffer!)
            
            var tagsByType = extractTagsByType(data)
            clinkboardDetected = clinkboardDetected && tagsByType[BOARD_3Part_CW] != nil && tagsByType[BOARD_3Part_CCW] != nil && tagsByType[BOARD_4Part_BB] != nil && tagsByType[BOARD_4Part_RR] != nil
            
            clinkcodeDetected = clinkcodeDetected && tagsByType[CODE_3Part_CW] != nil && tagsByType[CODE_3Part_CCW] != nil && tagsByType[CODE_3Part_CW]!.count > 1 && tagsByType[CODE_3Part_CW]!.count > 1
            
            if(clinkcodeDetected){
                let cwPairs = generateOppositeTagPairs(tagsByType[CODE_3Part_CW]!)
                let ccwPairs = generateOppositeTagPairs(tagsByType[CODE_3Part_CCW]!)
                
                clinkcodeDetected = false; //now we'll assume we didn't detect and see if we actually did
                if(cwPairs.count > 0 && ccwPairs.count > 0){
                    for p1 in cwPairs{
                        for p2 in ccwPairs{
                            if( p1.isCompatable(p2)){
                                let clinkcode = ClinkCode(cwPair: p1, ccwPair: p2, pixelBufferBaseAddress:pixelBufferPointer)
                                if(clinkcode.isValid){
                                    print("Found clinkcode: \(clinkcode.code)")
                                    clinkcodeDetected = true;
                                    hLine = Int32(clinkcode.centerXY.x)
                                    vLine = Int32(clinkcode.centerXY.y)
                                }
                            }
                        }
                    }
                }
            }
            
            //CVPixelBufferUnlockBaseAddress(imageBuffer!,[])
        }
        
        let t1 = CFAbsoluteTimeGetCurrent()
        //print("\(t1-t0)secs\n")
    }
    
    func drawAnchorGeometry(renderEncoder: MTLRenderCommandEncoder) {
        guard anchorInstanceCount - debugInstanceCount > 0 else { return }
        renderEncoder.pushDebugGroup("DrawAnchors")
        renderEncoder.setCullMode(.back)
        renderEncoder.setRenderPipelineState(anchorPipelineState)
        renderEncoder.setDepthStencilState(anchorDepthState)
        renderEncoder.setVertexBuffer(anchorUniformBuffer, offset: anchorUniformBufferOffset, index: 2)
        renderEncoder.setVertexBuffer(sharedUniformBuffer, offset: sharedUniformBufferOffset, index: 3)
        renderEncoder.setFragmentBuffer(sharedUniformBuffer, offset: sharedUniformBufferOffset, index: 3)
        for bufferIndex in 0..<mesh.vertexBuffers.count {
            let vertexBuffer = mesh.vertexBuffers[bufferIndex]
            renderEncoder.setVertexBuffer(vertexBuffer.buffer, offset: vertexBuffer.offset, index:bufferIndex)
        }
        for submesh in mesh.submeshes {
            renderEncoder.drawIndexedPrimitives(type: submesh.primitiveType, indexCount: submesh.indexCount, indexType: submesh.indexType, indexBuffer: submesh.indexBuffer.buffer, indexBufferOffset: submesh.indexBuffer.offset, instanceCount: anchorInstanceCount)
        }
        renderEncoder.popDebugGroup()
    }
    
    func drawDebugGeometry(renderEncoder: MTLRenderCommandEncoder) {
        guard debugInstanceCount > 0 else { return }
        renderEncoder.pushDebugGroup("DrawDebugPlanes")
        renderEncoder.setCullMode(.back)
        renderEncoder.setRenderPipelineState(debugPipelineState)
        renderEncoder.setDepthStencilState(debugDepthState)
        renderEncoder.setVertexBuffer(debugUniformBuffer, offset: debugUniformBufferOffset, index: 2)
        renderEncoder.setVertexBuffer(sharedUniformBuffer, offset: sharedUniformBufferOffset, index: 3)
        renderEncoder.setFragmentBuffer(sharedUniformBuffer, offset: sharedUniformBufferOffset, index: 3)
        for bufferIndex in 0..<debugMesh.vertexBuffers.count {
            let vertexBuffer = debugMesh.vertexBuffers[bufferIndex]
            renderEncoder.setVertexBuffer(vertexBuffer.buffer, offset: vertexBuffer.offset, index:bufferIndex)
        }
        for submesh in debugMesh.submeshes {
            renderEncoder.drawIndexedPrimitives(type: submesh.primitiveType, indexCount: submesh.indexCount, indexType: submesh.indexType, indexBuffer: submesh.indexBuffer.buffer, indexBufferOffset: submesh.indexBuffer.offset, instanceCount: debugInstanceCount)
        }
        renderEncoder.popDebugGroup()
    }
}

/* Helper Functions */

func generateOppositeTagPairs(_ fromCodeTags:[ExtractedTag] ) -> [OppositeTagPair]{
    var pairs:[OppositeTagPair] = []
    
    for (i,tag) in fromCodeTags.enumerated(){
        let oppTags = findPossibleOppositeCornerTags(codeTag:tag, possibleMatches:fromCodeTags, startAt:i+1 )
        for oppTag in oppTags{
            pairs.append(OppositeTagPair(tagA:tag,tagB:oppTag))
        }
    }
    return pairs
}

func findPossibleOppositeCornerTags( codeTag:ExtractedTag, possibleMatches:[ExtractedTag], startAt:Int ) -> [ExtractedTag] {
    var matches:[(tag:ExtractedTag,mag:Double)] = []
    let orientDiffThresh = codeTag.orientHypot/1.5;
    if(startAt < possibleMatches.count){
        for tag in possibleMatches[startAt...]{
            guard tag.cellIndex != codeTag.cellIndex else{
                continue
            }
            let sumX = codeTag.orientX + tag.orientX
            let sumY = codeTag.orientY + tag.orientY
            let mag = hypot(sumX,sumY)
            if(mag < orientDiffThresh){
                let i = matches.index(where: { (m) -> Bool in m.mag > mag })
                if(i == nil){
                    matches.append((tag:tag, mag:mag))
                }
                else{
                    matches.insert((tag:tag, mag:mag), at: i!)
                }
            }
        }
    }
    return matches.map {$0.tag}
}

func extractTagsByType(_ data:UnsafeMutablePointer<Int32> ) -> [Int:[ExtractedTag]] {
    var tagsByType:[Int:[ExtractedTag]] = [:]
    for n in 0..<numGridCells {
        let offset = numTagTypes + valuesPerCell*n
        let weight:Double = Double(data[offset + TOTAL_WEIGHT])
        if(weight > 0){
            let typeTotal:Double = Double(data[offset + TYPE_AVERAGE])
            let type:Int = Int(round(typeTotal / weight))
            if(isClinkcodeType(type: type) || isClinkboardType(type: type)){
                let typeFlags:Int32 = data[offset + TYPE_FLAGS]
                let xCoord:Double = Double(data[offset + X_COORD_AVERAGE]) / weight
                let yCoord:Double = Double(data[offset + Y_COORD_AVERAGE]) / weight
                let orientX:Double = Double(data[offset + X_ORIENTATION_AVERAGE]) / weight
                let orientY:Double = Double(data[offset + Y_ORIENTATION_AVERAGE]) / weight
                let dotSize:Double = Double(data[offset + DOT_SIZE]) / weight
                let extractedTag = ExtractedTag(
                    weight:weight,
                    typeFlags:typeFlags,
                    type:type,
                    x:xCoord,
                    y:yCoord,
                    orientX:orientX,
                    orientY:orientY,
                    orientHypot:hypot(orientX,orientY),
                    dotSize:dotSize,
                    cellIndex:n)
                if(tagsByType[type] == nil){
                    tagsByType[type] = [extractedTag]
                }
                else{
                    tagsByType[type]!.append(extractedTag)
                }
            }
        }
    }
    return tagsByType
}

func sortTagsByWeight(_ tagArray:[ExtractedTag]) -> [ExtractedTag]{
    var tags:[ExtractedTag] = []
    for tag in tagArray{
        let i = tags.index(where: { (t) -> Bool in t.weight < tag.weight })
        if(i == nil){
            tags.append(tag)
        }
        else{
            tags.insert(tag, at: i!)
        }
    }
    return tags
}

func buildClinkcodeProjectionMatrix(_ clinkcode :  inout ClinkCode ) -> [Double] {
    let numBlocks:Double = 6
    //print("buildMatrix(-1,-1,\(clinkcode.topLeft.x),\(clinkcode.topLeft.y),\(numBlocks),-1,\(clinkcode.topRight.x),\(clinkcode.topRight.y),-1,\(numBlocks),\(clinkcode.bottomLeft.x),\(clinkcode.bottomLeft.y),\(numBlocks),\(numBlocks),\(clinkcode.bottomRight.x),\(clinkcode.bottomRight.y))")
    let s = formBasis(p1:(x:-1, y:-1), p2:(x:numBlocks, y:-1), p3:(x:-1, y:numBlocks), p4:(x:numBlocks, y:numBlocks))
    let d = formBasis(p1:clinkcode.topLeft.xy, p2:clinkcode.topRight.xy, p3:clinkcode.bottomLeft.xy, p4:clinkcode.bottomRight.xy)
    var t = multiplyMatrices3(a:d, b:adjugate3(s))
    
    for i in 0..<9{
        t[i] = t[i]/t[8]
    }
    
    clinkcode.projectionMatrix = [
        t[0], t[3], 0, t[6],
        t[1], t[4], 0, t[7],
        0   , 0   , 1, 0   ,
        t[2], t[5], 0, t[8]]
    
    //print("Matrix: \(clinkcode.projectionMatrix)")
    return clinkcode.projectionMatrix
}

func formBasis( p1:(x:Double, y:Double), p2:(x:Double, y:Double), p3:(x:Double, y:Double), p4:(x:Double, y:Double) ) -> [Double] {
    let m:[Double] = [
        p1.x, p2.x, p3.x,
        p1.y, p2.y, p3.y,
        1,  1,  1 ]
    let pt4:[Double] = [p4.x, p4.y, 1]
    var v = multiplyMatrixAndVector3( m:adjugate3(m), v:pt4)
    return multiplyMatrices3(a:m, b:[
        v[0], 0, 0,
        0, v[1], 0,
        0, 0, v[2]
        ])
}


func adjugate3(_ m:[Double] ) -> [Double] {
    return [
        m[4]*m[8]-m[5]*m[7], m[2]*m[7]-m[1]*m[8], m[1]*m[5]-m[2]*m[4],
        m[5]*m[6]-m[3]*m[8], m[0]*m[8]-m[2]*m[6], m[2]*m[3]-m[0]*m[5],
        m[3]*m[7]-m[4]*m[6], m[1]*m[6]-m[0]*m[7], m[0]*m[4]-m[1]*m[3]
    ]
}

func multiplyMatrices3( a:[Double], b:[Double]) -> [Double] {
    var c:[Double] = [0,0,0,0,0,0,0,0,0]
    for i in 0..<3{
        for j in 0..<3{
            var cij:Double = 0
            for k in 0..<3{
                cij += a[3*i + k]*b[3*k + j]
            }
            c[3*i + j] = cij
        }
    }
    return c
}

func multiplyMatrixAndVector3(m:[Double], v:[Double]) -> [Double] {
    return [
        m[0]*v[0] + m[1]*v[1] + m[2]*v[2],
        m[3]*v[0] + m[4]*v[1] + m[5]*v[2],
        m[6]*v[0] + m[7]*v[1] + m[8]*v[2]
    ]
}

func projectPoint(m:[Double], pt:(x:Double, y:Double)) -> (x:Double, y:Double) {
    let s:Double = 1/(m[ 3 ] * pt.x + m[ 7 ] * pt.y + m[ 15 ])
    let x:Double = s*(m[ 0 ] * pt.x + m[ 4 ] * pt.y + m[ 12 ])
    let y:Double = s*(m[ 1 ] * pt.x + m[ 5 ] * pt.y + m[ 13 ])
    return (x:x, y:y)
}
