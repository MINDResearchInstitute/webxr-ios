import MetalKit
import ARKit
import JavaScriptCore
import CoreLocation

/*
 TODO: A lot of the code here should probably be moved out:
 controller stuff into a controller class
 helper functions and stuff into a helper library
 */

let sizeInt32 = MemoryLayout<Int32>.stride

protocol RenderDestinationProvider {
    var currentRenderPassDescriptor: MTLRenderPassDescriptor? { get }
    var currentDrawable: CAMetalDrawable? { get }
    var colorPixelFormat: MTLPixelFormat { get set }
    var sampleCount: Int { get set }
}

let maxBuffersInFlight: Int = 1

//Per-Cell ClinkData Fields
let valuesPerCell = 8

//ClinkData Field IDs:
let TOTAL_WEIGHT = 0
let TYPE_FLAGS = 1
let TYPE_AVERAGE = 2
let X_COORD_AVERAGE = 3
let Y_COORD_AVERAGE = 4
let X_ORIENTATION_AVERAGE = 5
let Y_ORIENTATION_AVERAGE = 6
let DOT_SIZE = 7

let NUM_PIX_X = 1920
let NUM_PIX_Y = 1080
let halfPixWidth = Double(NUM_PIX_X) / 2.0
let halfPixHeight = Double(NUM_PIX_Y) / 2.0
let SCREEN_CENTER = [halfPixWidth,halfPixHeight,0]
let BYTES_PER_ROW = NUM_PIX_X * 4
let GRID_RESOLUTION = 1
var gridDivisionsX = 16*GRID_RESOLUTION
var gridDivisionsY = 9*GRID_RESOLUTION
var clinkDataSize = valuesPerCell*gridDivisionsX*gridDivisionsY + numTagTypes
var numGridCells = gridDivisionsX*gridDivisionsY

let locationUpdateInterval:Double = 15*60*1000 //update location every 15 minutes

//Debug lines. We can remove later:
var hLine:Int32 = Int32(NUM_PIX_X/2)
var vLine:Int32 = Int32(NUM_PIX_Y/2)

//ClinkCorner Types
let numTagTypes = 30
////// TYPES  ///////
let BOARD_3Part_CW = 0
let CODE_3Part_CW = 1
let BOARD_3Part_CCW = 2
let CODE_3Part_CCW = 3
let BOARD_4Part_RR = 4
let MARKER_4Part_YY = 5
let BOARD_4Part_BB = 6

let VALID_CLINKCODE_DIAGONALS = [28, 23, 49, 19, 52, 46, 13, 59]

let LOW_ERROR_POSE_THRESHOLD:Double = 12.0
let CENTER_DIST_THRESHOLD:Double = Double(NUM_PIX_Y) * 0.4
let MIN_DEVICE_MOTION_FOR_TRIANGULATION:Double = 0.1
let TRIANGULATION_ERROR_THRESHOLD:Double = 10


//JAVASCRIPT RESOURCES
let javascriptResourcesToLoad = ["MINDPositSVD","MINDMath","MINDXR"]
//Javascript Stuff. This is a singleton
var jsContext:JSContext?
var clinkcodePoseFunction:JSValue?
var triangulatePointsFunction:JSValue?
var clinkcodeTestFunction:JSValue?

/* ************************** */
/*      MINDMetalRenderer     */
/* ************************** */

@objc class MINDMetalRenderer:NSObject, CLLocationManagerDelegate {
    
    //Rendering Stuff
    var viewportSize: CGSize = CGSize()
    var viewportSizeDidChange: Bool = false
    var viewportOrientation:UIInterfaceOrientation = .portrait
    var session: ARSession!
    var lastFrame:ARFrame!
    var device: MTLDevice!
    var commandQueue: MTLCommandQueue!
    let inFlightSemaphore = DispatchSemaphore(value: maxBuffersInFlight)
    var renderDestination: MTKView!
    var sharedUniformBuffer: MTLBuffer!
    var imagePlaneVertexBuffer: MTLBuffer!
    var convertToRGBPipelineState: MTLRenderPipelineState!
    var capturedImagePipelineState: MTLRenderPipelineState!
    var capturedImageTextureY: MTLTexture!
    var capturedImageTextureCbCr: MTLTexture!
    var convertedImageTextureRGB: MTLTexture!
    var conversionPassDescriptor: MTLRenderPassDescriptor!
    var capturedImageTextureCache: CVMetalTextureCache!
    var uniformBufferIndex: Int = 0
    var sharedUniformBufferOffset: Int = 0
    var sharedUniformBufferAddress: UnsafeMutableRawPointer!
    var capturedImagePixelBuffer:CVPixelBuffer!
    
    let locationManager = CLLocationManager()
    var locationUpdateTimer:Timer?
    var latestLocation:[Double]? //[latitude, longitude, altitudeInMeters]
    
    //Camera Stuff
    var backCamera:AVCaptureDevice!
    var focalLength:Double = 1572 //we periodically update below from ARCamera.intrinsics
    var fieldOfViewVDeg:Double = 37.9150085 //Vertical FieldOfView in degrees
    var fieldOfViewHDeg:Double = 67.4044647 //Horizontal FieldOfView in degrees
    
    //Clink Database
    var clinkFrameID:Int = 0
    var clinkcodeDB:[Int32:ClinkcodeDataSet] = [:]
    var newClinkcodeNotificationRequested:Bool = true
    //var encounteredClinkboards:[Int:ClinkBoard] = [:]
    
    struct SharedUniforms {
        var projectionMatrix: matrix_float4x4
        var viewMatrix: matrix_float4x4
    }
    
    let alignedSharedUniformSize = (MemoryLayout<SharedUniforms>.size & ~0xFF) + 0x100
    
    let planeVertexData: [Float] = [-1, -1,  0,  1, 1, -1,  1,  1, -1,  1,  0,  0, 1,  1,  1,  0]
    
    //Clink Data
    internal var clinkData = [Int32](repeating:0, count: clinkDataSize)
    internal var clinkDataBuffer: MTLBuffer?
    
    @objc var clinkFrame:Dictionary<String, Any> = [:];
    @objc var clinkInfo:Dictionary<String, Any> = [:];
    
    @objc func setup(session: ARSession, device: MTLDevice, view: MTKView) {
        self.session = session
        self.device = device
        self.renderDestination = view
        backCamera = AVCaptureDevice.default(.builtInWideAngleCamera, for: AVMediaType.video, position: .back)
        setupJavascript()
        setupPipeline()
        
        //TODO: If the ARKit is already tracking and updating location, we don't need to do this:
        locationManager.delegate = self
        locationManager.requestLocation()
        locationUpdateTimer = Timer.scheduledTimer(withTimeInterval:locationUpdateInterval, repeats: true) { (timer) in
            self.locationManager.requestLocation()
        }
    }
    
    func setupJavascript(){
        if(jsContext == nil){
            let contextName = "MINDXR_JSContext"
            jsContext = JSContext()
            jsContext!.name = contextName
            jsContext!.exceptionHandler = { context, exception in
                print("JSContext( \(contextName) Error: \(exception!)")
            }
            
            jsContext!.evaluateScript("var console = { log: function(message) { _consoleLog(message) } }")
            let consoleLog: @convention(block) (String) -> Void = { message in
                print("jsconsole.log: " + message)
            }
            jsContext!.setObject(unsafeBitCast(consoleLog, to: AnyObject.self), forKeyedSubscript: "_consoleLog" as (NSCopying & NSObjectProtocol))
            
            //Load Javscript Resources:
            for jsResourceName in javascriptResourcesToLoad {
                if let path = Bundle.main.path(forResource: jsResourceName, ofType: "js"),
                    let jsSource = try? String(contentsOfFile: path) {
                    let result = jsContext!.evaluateScript(jsSource)!
                    print("Loaded \(jsResourceName) with result: \(result)")
                }
            }
            
            //TODO: If the focalLength changes, we'll want to rebuild the posit
            //context.evaluateScript("clinkcodePosit = new POS.Posit(1, \(focalLength));")
            clinkcodePoseFunction = jsContext!.objectForKeyedSubscript("estimateClinkcodePose")
            triangulatePointsFunction = jsContext!.objectForKeyedSubscript("triangulatePointsFrom2CameraViews")
        }
    }
    
    //testing the flashlight. Will be turning this into a very brief pulse to detect when using the AR_Mirror
    func pulseFlash(_ on: Bool) {
        if backCamera.hasTorch && backCamera.isTorchAvailable {
            do {
                try backCamera.lockForConfiguration()
                if on == true {
                    backCamera.torchMode = .on // set on
                } else {
                    backCamera.torchMode = .off // set off
                }
                backCamera.unlockForConfiguration()
            } catch {
                print("Flash could not be used")
            }
        } else {
            print("Flash is not available")
        }
    }
    
    //NOTE: Can we use this to see if we are focusing on something near or far?
    func getLenseInfo() -> (position:Float, aperture:Float ) {
        return (position:backCamera.lensPosition, aperture:backCamera.lensAperture)
    }
    
    func setupPipeline() {
        renderDestination.colorPixelFormat = .bgra8Unorm
        renderDestination.sampleCount = 1
        
        //Shaders
        let defaultLibrary = device.makeDefaultLibrary()!
        let directVertexShader = defaultLibrary.makeFunction(name: "directVertexShader")
        let transformingVertexShader = defaultLibrary.makeFunction(name: "transformingVertexShader")
        let convertToRGBFragmentShader = defaultLibrary.makeFunction(name: "ycbcrToRGBFragmentShader")
        let capturedImageFragmentShader = defaultLibrary.makeFunction(name: "clinkFragmentShader")
        
        //BUFFERS and TEXTURES
        //Intermediate Texture to store RGB texture and pass into clink shader
        let textureDescriptor = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .bgra8Unorm, width: NUM_PIX_X, height: NUM_PIX_Y, mipmapped: false)
        textureDescriptor.usage = [.shaderRead, .shaderWrite, .renderTarget]
        convertedImageTextureRGB = device.makeTexture(descriptor: textureDescriptor)!
        let sharedUniformBufferSize = alignedSharedUniformSize * maxBuffersInFlight
        sharedUniformBuffer = device.makeBuffer(length: sharedUniformBufferSize, options: .storageModeShared)
        let imagePlaneVertexDataCount = planeVertexData.count * MemoryLayout<Float>.size
        imagePlaneVertexBuffer = device.makeBuffer(bytes: planeVertexData, length: imagePlaneVertexDataCount, options: [])
        var textureCache: CVMetalTextureCache?
        CVMetalTextureCacheCreate(nil, nil, device, nil, &textureCache)
        capturedImageTextureCache = textureCache
        clinkDataBuffer = device.makeBuffer(bytes: &clinkData, length: clinkDataSize * sizeInt32, options: .storageModeShared)
        
        //Image Plane
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
        
        conversionPassDescriptor = MTLRenderPassDescriptor()
        conversionPassDescriptor.colorAttachments[0].clearColor = MTLClearColorMake(1, 1, 1, 1)
        conversionPassDescriptor.colorAttachments[0].loadAction = .load
        conversionPassDescriptor.colorAttachments[0].storeAction = .store
        conversionPassDescriptor.colorAttachments[0].texture = convertedImageTextureRGB
        
        let convertToRGBPipelineDescriptor = MTLRenderPipelineDescriptor()
        convertToRGBPipelineDescriptor.label = "ConvertToRGBPipeline"
        convertToRGBPipelineDescriptor.vertexFunction = directVertexShader
        convertToRGBPipelineDescriptor.fragmentFunction = convertToRGBFragmentShader
        convertToRGBPipelineDescriptor.vertexDescriptor = imagePlaneVertexDescriptor
        convertToRGBPipelineDescriptor.sampleCount = 1
        convertToRGBPipelineDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
        do { try convertToRGBPipelineState = device.makeRenderPipelineState(descriptor: convertToRGBPipelineDescriptor) }
        catch let error { print("Failed to created convertToRGB pipeline state, error \(error)") }
        
        let clinkPipelineStateDescriptor = MTLRenderPipelineDescriptor()
        clinkPipelineStateDescriptor.label = "MyCapturedImagePipeline"
        clinkPipelineStateDescriptor.sampleCount = renderDestination.sampleCount
        clinkPipelineStateDescriptor.vertexFunction = transformingVertexShader
        clinkPipelineStateDescriptor.fragmentFunction = capturedImageFragmentShader
        clinkPipelineStateDescriptor.vertexDescriptor = imagePlaneVertexDescriptor
        clinkPipelineStateDescriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        do { try capturedImagePipelineState = device.makeRenderPipelineState(descriptor: clinkPipelineStateDescriptor) }
        catch let error { print("Failed to created captured clink pipeline state, error \(error)") }
        
        commandQueue = device.makeCommandQueue()
    }
    
    @objc func drawRectResized(size: CGSize) {
        viewportSize = size
        viewportSizeDidChange = true
    }
    
    @objc func update() {
        let pixBuff = capturedImagePixelBuffer
        
        let _ = inFlightSemaphore.wait(timeout: DispatchTime.distantFuture)
        guard let commandBuffer = commandQueue.makeCommandBuffer() else { return }
        commandBuffer.addCompletedHandler{ [weak self] commandBuffer in
            if let strongSelf = self { strongSelf.inFlightSemaphore.signal() }
            if(pixBuff != nil){
                let pixelBufferBaseAddress:UnsafeMutableRawPointer = CVPixelBufferGetBaseAddressOfPlane(pixBuff!,0)!
                self!.processClinkData( pixelBufferBaseAddress )
                CVPixelBufferUnlockBaseAddress(pixBuff!,.readOnly)
            }
            return
        }
        updateBufferStates()
        updateWorldState()
        
        //Clear the clinkDataBuffer
        guard let clinkDataBuffer = clinkDataBuffer else { return }
        let blitEncoder = commandBuffer.makeBlitCommandEncoder()!
        blitEncoder.fill(buffer: clinkDataBuffer, range:0..<clinkDataBuffer.length, value:0)
        blitEncoder.endEncoding()
        
        if(pixBuff != nil){
            CVPixelBufferLockBaseAddress(pixBuff!,.readOnly)
        }
        
        guard let conversionRenderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: conversionPassDescriptor) else { return }
        convertTextureToRGB(renderEncoder: conversionRenderEncoder)
        conversionRenderEncoder.endEncoding()
        
        guard let renderPassDescriptor = renderDestination.currentRenderPassDescriptor,
            let drawable = renderDestination.currentDrawable else { return }
        guard let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else { return }
        detectAndRenderClinkData(renderEncoder: renderEncoder)
        
        renderEncoder.endEncoding()
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }
    
    func updateBufferStates() {
        uniformBufferIndex = (uniformBufferIndex + 1) % maxBuffersInFlight
        sharedUniformBufferOffset = alignedSharedUniformSize * uniformBufferIndex
        sharedUniformBufferAddress = sharedUniformBuffer.contents().advanced(by: sharedUniformBufferOffset)
    }
    
    func updateWorldState() {
        guard let currentFrame = session.currentFrame else { return }
        
        //TODO: We'll want to pass the projectionMatrix and FieldOfView info on to javascript so ThreeJS can match camera
        let projectionMatrix = currentFrame.camera.projectionMatrix
        
        //Calculate the FieldOfView FOV for vertical and horizontal
        let imageResolution = currentFrame.camera.imageResolution
        let intrinsics = currentFrame.camera.intrinsics
        let newFocalLength:Double = round(Double(intrinsics[0,0]))
        if(self.focalLength != newFocalLength){
            //TODO: We'll want to update the posit and other things that use this value
            //let newHFieldOfView = 2 * atan(Float(imageResolution.width)/(2 * intrinsics[0,0])) * 180/Float.pi
            //let newVFieldOfView = 2 * atan(Float(imageResolution.height)/(2 * intrinsics[1,1])) * 180/Float.pi
            
            //A different way of calculating the above values. They should be the same!
            let yFovDeg = 2 * atan(1/projectionMatrix[1,1]) * 180/Float.pi
            let xFovDeg = yFovDeg * Float(imageResolution.width / imageResolution.height)
            
            self.focalLength = newFocalLength
            self.fieldOfViewVDeg = Double(yFovDeg)
            self.fieldOfViewHDeg = Double(xFovDeg)
        }
        
        updateSharedUniforms(frame: currentFrame)
        updateCapturedImageTextures(frame: currentFrame)
        if viewportSizeDidChange {
            viewportSizeDidChange = false
            updateImagePlane(frame: currentFrame)
        }
    }
    
    func updateSharedUniforms(frame: ARFrame) {
        //We don't really need these right now
        let uniforms = sharedUniformBufferAddress.assumingMemoryBound(to: SharedUniforms.self)
        uniforms.pointee.viewMatrix = simd_inverse(frame.camera.transform)
        uniforms.pointee.projectionMatrix = frame.camera.projectionMatrix(for: viewportOrientation, viewportSize: viewportSize, zNear: 0.001, zFar: 1000)
    }
    
    func updateCapturedImageTextures(frame: ARFrame) {
        capturedImagePixelBuffer = frame.capturedImage
        if (CVPixelBufferGetPlaneCount(capturedImagePixelBuffer) < 2) { return }
        capturedImageTextureY = createTexture(fromPixelBuffer: capturedImagePixelBuffer, pixelFormat:.r8Unorm, planeIndex:0)!
        capturedImageTextureCbCr = createTexture(fromPixelBuffer: capturedImagePixelBuffer, pixelFormat:.rg8Unorm, planeIndex:1)!
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
        let displayToCameraTransform = frame.displayTransform(for: viewportOrientation, viewportSize: viewportSize).inverted()
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
        renderEncoder.setVertexBuffer(imagePlaneVertexBuffer, offset: 0, index: 0)
        renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
        renderEncoder.popDebugGroup()
    }
    
    func detectAndRenderClinkData(renderEncoder: MTLRenderCommandEncoder ) {
        guard capturedImageTextureY != nil && capturedImageTextureCbCr != nil else { return }
        renderEncoder.pushDebugGroup("detectAndRenderClinkData")
        renderEncoder.setCullMode(.none)
        renderEncoder.setRenderPipelineState(capturedImagePipelineState)
        renderEncoder.setVertexBuffer(imagePlaneVertexBuffer, offset: 0, index: 0)
        renderEncoder.setFragmentTexture(convertedImageTextureRGB, index: 0)
        renderEncoder.setFragmentBuffer(clinkDataBuffer, offset: 0, index: 0)
        renderEncoder.setFragmentBytes(&hLine, length: sizeInt32, index: 1)
        renderEncoder.setFragmentBytes(&vLine, length: sizeInt32, index: 2)
        renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
        renderEncoder.popDebugGroup()
        
    }
    
    func processClinkData(_ pixelBufferBaseAddress:UnsafeMutableRawPointer ){
        clinkFrameID = clinkFrameID + 1
        
        hLine = Int32(0)
        vLine = Int32(0)
        
        let data = (clinkDataBuffer?.contents().bindMemory(to: Int32.self, capacity: clinkDataSize))!
        
        var clinkboardDetected = (data[BOARD_4Part_RR] > 0 && data[BOARD_4Part_BB] > 0 && data[BOARD_3Part_CW] > 0 && data[BOARD_3Part_CCW] > 0)
        var clinkcodeDetected = (data[CODE_3Part_CW] > 1 && data[CODE_3Part_CCW] > 1)
        
        if(clinkboardDetected || clinkcodeDetected) {
            guard let currentFrame = session.currentFrame else { return }
            if(lastFrame == currentFrame){
                return
            }
            lastFrame = currentFrame
            
            let lightEst = Double((currentFrame.lightEstimate?.ambientIntensity)!)
            let timestamp = Date().timeIntervalSince1970 as Double
            
            if(latestLocation == nil ){
                let currentLocation = locationManager.location
                if(currentLocation != nil){
                    let coord = currentLocation?.coordinate
                    let alt = currentLocation?.altitude
                    latestLocation = [Double(coord?.latitude ?? 0), Double(coord?.longitude ?? 0), Double(alt ?? 0)]
                }
            }
            
            let cameraPose = flattenMatrix(currentFrame.camera.transform)
            let cameraProjMat = flattenMatrix(currentFrame.camera.projectionMatrix)
            let lenseInfo = getLenseInfo()
            
            var tagsByType = extractTagsByType(data)
            
            let clinkDataFrame = ClinkDataFrame(frameID:clinkFrameID, timestamp:timestamp, location:latestLocation ?? [], focalLength:focalLength, cameraPose:cameraPose, cameraProjection:cameraProjMat, lenseInfo:lenseInfo, tagsInFrame:tagsByType, lightingEstimate:lightEst)
            
            clinkboardDetected = clinkboardDetected && tagsByType[BOARD_3Part_CW] != nil && tagsByType[BOARD_3Part_CCW] != nil && tagsByType[BOARD_4Part_BB] != nil && tagsByType[BOARD_4Part_RR] != nil
            
            clinkcodeDetected = clinkcodeDetected && tagsByType[CODE_3Part_CW] != nil && tagsByType[CODE_3Part_CCW] != nil && tagsByType[CODE_3Part_CW]!.count > 1 && tagsByType[CODE_3Part_CW]!.count > 1
            
            if(clinkcodeDetected){
                let cwPairs = generateOppositeTagPairs(tagsByType[CODE_3Part_CW]!)
                let ccwPairs = generateOppositeTagPairs(tagsByType[CODE_3Part_CCW]!)
                
                if(cwPairs.count > 0 && ccwPairs.count > 0){
                    for p1 in cwPairs{
                        for p2 in ccwPairs{
                            guard p1.isCompatable(p2) else {continue}
                            let clinkcode = Clinkcode(cwPair: p1, ccwPair: p2, pixelBufferBaseAddress:pixelBufferBaseAddress, dataFrame:clinkDataFrame)
                            guard clinkcode.isValid else {continue}
                            clinkcodeFound(clinkcode)
                        }
                    }
                }
            }
        }
    }
    
    func clinkcodeFound(_ clinkcode:Clinkcode){
        let code:Int32 = clinkcode.code
        guard code > 0 else {return}
        
        //Debug stuff:
        //print("Found clinkcode: \(clinkcode.code)") //debug code is: 10894314
        hLine = Int32(clinkcode.centerXY.x)
        vLine = Int32(clinkcode.centerXY.y)
        
        var clinkcodeDataSet = clinkcodeDB[code]
        if(clinkcodeDataSet == nil){
            if(clinkcode.hasLowErrorPose()){
                clinkcodeDataSet = ClinkcodeDataSet(code);
                clinkcodeDB[code] = clinkcodeDataSet
            }
        }
        guard clinkcodeDataSet != nil else { return }
        clinkcodeDataSet!.newEncounter(clinkcode)
        for newInstance in clinkcodeDataSet!.newInstances{
            newClinkcodeInstanceEncountered(newInstance)
        }
        
        for triangulatedInstance in clinkcodeDataSet!.newTriangulations{
            newClinkcodeTriangulation(triangulatedInstance)
        }
        clinkcodeDataSet!.newInstances.removeAll()
        clinkcodeDataSet!.newTriangulations.removeAll()
    }
    
    func newClinkcodeInstanceEncountered(_ instance:ClinkcodeInstanceData ){
        var newClinkcodeInstances = clinkInfo["newClinkcodeInstances"] as? [Dictionary<String, Any>]
        if(newClinkcodeInstances == nil){
            newClinkcodeInstances = []
        }
        
        newClinkcodeInstances!.append(getClinkcodeInstanceJSON(instance))
        clinkInfo["newClinkcodeInstances"] = newClinkcodeInstances!
    }
    
    func newClinkcodeTriangulation(_ instance:ClinkcodeInstanceData ){
        var newTriangulations = clinkInfo["newClinkcodeTriangulations"] as? [Dictionary<String, Any>]
        if(newTriangulations == nil){
            newTriangulations = []
        }
        newTriangulations!.append(getClinkcodeInstanceJSON(instance))
        clinkInfo["newClinkcodeTriangulations"] = newTriangulations!
    }
    
    func getClinkcodeInstanceJSON(_ instance:ClinkcodeInstanceData) -> Dictionary<String, Any> {
        return [
            "code" : instance.code,
            "id" : instance.instanceID,
            "room" : instance.room?.clinkroomID ?? "",
            "gps" : instance.seedEncounter?.clinkDataFrame.location ?? locationManager.location ?? [],
            "tilt" : instance.seedEncounter?.getTilt() ?? [],
            "tagSize" : instance.tagSize,
            "tagModel" : instance.tagModel,
            "localCoordinates" : instance.localCoordinates
        ]
    }
    
    //Location Manager Delegate Functions:
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        if let location = locations.first {
            print("Updated user's location: \(location)")
            latestLocation = nil //force an update
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        print("Failed to update user's location: \(error.localizedDescription)")
    }
}

/* *************************** */
/*    STRUCTs Classes Enums   */
/* *************************** */

enum ClinkcodeEncounterEvent {
    case None
    case ReferenceSet
    case TagSizeTriangulated
}

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


struct ClinkDataFrame{
    let frameID:Int
    let timestamp:Double
    let location:[Double]
    let focalLength:Double
    let cameraPose:[Double]
    let cameraProjection:[Double]
    let lenseInfo:(position:Float, aperture:Float )
    let tagsInFrame:[Int:[ExtractedTag]] //can be used to distinguish multiple instances of same clinkdata
    let lightingEstimate:Double
    
    func getCameraLoc() -> [Double]{
        return [cameraPose[12],cameraPose[13],cameraPose[14]]
    }
    //gps location info
}

struct TriangulatedClinkcodeData{
    let frame:ClinkDataFrame
    let error:Double
    let tagSize:Double
    let centerPoint:[Double]
    let topLeftPoint:[Double]
    let topRightPoint:[Double]
    let bottomRightPoint:[Double]
    let bottomLeftPoint:[Double]
    let tilt:[Double]
}

//TODO: Detect 4 code tags that are next to each other, but belong to different clinkcodes
//      We use this to detect and track multidisplays
struct InterTaggalSpace{
    let cornerTags:[ExtractedTag]
    let centerXY:(x:Double, y:Double)
    let frame:ClinkDataFrame
}

class ClinkRoom{
    let clinkroomID:String = ""
    let globalPosition:[Double] = []
    let sessionCoordinates:[Double] = [] //position of room's origin in current ARSession coordinates
    let sessionMatrix:[Double] = [] //transform matrix for the room in session coordinates
    let tilt:[Double] = [] //[upwardpitch, compassYawNESW, zRoll]  in degrees. pitch and roll are usually zero
    let roomType:Int = -1 // type of room. Classroom, remoteStudentRoom...
    let roomDescription:String = "" //room name or number or both, could also have building info, floor info, street address
    let elevation:Double = 0 //elevation of the room off of the outside ground. Could be high for skyscrapers
    let floorOffset:Double = 0 //offset of the floor from the rooms origin at the origin. Could have different offsets other places
    let anchors:[ClinkRoomAnchor] = [] //list of anchors with anchor info
    let anchorGrups:[ClinkRoomAnchorGroup] = []
    let clinkCodeInstances:[ClinkcodeInstanceData] = [] //clinkcode instances found in the room
}

class ClinkRoomAnchor{
    var parentRoom:ClinkRoom?
    var parentGroup:ClinkRoomAnchorGroup?
    let anchorType:String = "" //type of anchor: floor point, ceiling point, display point, lighting
    let anchorSubindex:Int = 0 //index within a group
    let localCoordinates:[Double] = [] //location in room coordinates
    let sessionCoordinates:[Double] = [] //position in current ARSession coordinates
}

class ClinkRoomAnchorGroup{
    var parentRoom:ClinkRoom?
    let groupType:String = "" //floor, ceiling, wall, display, etc
    let anchorGroupName:String = "" //name of the group
    let anchors:[ClinkRoomAnchor] = [] //anchors part of this group
}

class ClinkDisplay{
    let clinkRoom:ClinkRoom
    var tagSize:Double
    
    init( room:ClinkRoom, tagSize:Double ){
        clinkRoom = room
        self.tagSize = tagSize
    }
}

//Need to have MultiDisplay and SingleDisplay types

//Location Types:
//sessionCoordinates: location in current-session device coordinates
//localCoordinates: location in room coordinates
//globalPosition: GPS coordinates

class ClinkcodeInstanceData{
    let code:Int32
    var instanceID:String = ""
    
    var room:ClinkRoom?
    var seedEncounter:Clinkcode?
    var referenceEncounter:Clinkcode?
    var latestEncounter:Clinkcode?
    
    var tagSize:Double = 0
    var tagModel:[Double] = [] //[] means the default model: [-0.5,0.5,0,  0.5,0.5,0,  0.5,-0.5,0,  -0.5,-0.5,0]
    var sessionCoordinates:[Double] = []
    var localCoordinates:[Double] = []
    var tilt:[Double]
    
    var triangulationSamples:[TriangulatedClinkcodeData] = []
    
    init(_ seedClinkcode:Clinkcode ){
        self.code = seedClinkcode.code
        self.seedEncounter = seedClinkcode
        self.referenceEncounter = seedClinkcode
        self.tilt = seedClinkcode.getTilt()!
    }
    
    func newEncounter(_ encounter:Clinkcode ) -> ClinkcodeEncounterEvent{
        if(tagSize == 0){
            if(referenceEncounter == nil){
                referenceEncounter = encounter
                return .ReferenceSet
            }
            else{
                let refDeviceLoc = referenceEncounter!.clinkDataFrame.getCameraLoc()
                let newDeviceLoc = encounter.clinkDataFrame.getCameraLoc()
                let deviceMotion = distanceBetween(refDeviceLoc, newDeviceLoc)
                if(deviceMotion > MIN_DEVICE_MOTION_FOR_TRIANGULATION && encounter.isNearCenter()){
                    guard let triangulationData = triangulateTagSize(referenceEncounter!, encounter),
                        triangulationData.error < TRIANGULATION_ERROR_THRESHOLD
                        else {return .None}
                    referenceEncounter = nil
                    triangulationSamples.append(triangulationData)
                    if(triangulationSamples.count > 2){
                        var avgTagSize:Double = 0
                        for tsamp in triangulationSamples{
                            avgTagSize += tsamp.tagSize
                        }
                        avgTagSize /= Double(triangulationSamples.count)
                        self.tagSize = avgTagSize
                        return .TagSizeTriangulated
                    }
                    
                }
            }
        }
        return .None
    }
    
    func isPossibleMatch(_ forClinkcode:Clinkcode) -> Bool{
        guard let tiltB = forClinkcode.getTilt() else {return false}
        let tiltA = tilt
        let upwardPitchDiff = abs(tiltB[0] - tiltA[0])
        guard upwardPitchDiff < 10 else {return false}
        if(abs(tilt[0]) < 45){
            //The tag is upright
            let compassDiff = getDegreeDifference(tiltB[1], tiltA[1])
            guard compassDiff < 20 else {return false} //extra room for error in compass calibration
            let zRollDiff = abs(tiltB[2] - tiltA[2])
            guard zRollDiff < 10 else {return false}
            return true
        }
        else{
            //TODO: This is not good because the old seedEncounter might need to be updated.
            guard let prevEncounter = referenceEncounter ?? seedEncounter else {return false}
            let seedQuat = prevEncounter.getGlobalQuaternion()!
            let newQuat = forClinkcode.getGlobalQuaternion()!
            let quatDiff = getAngleBetweenQuaternions(seedQuat,newQuat)
            return quatDiff < 0.3
        }
    }
}

class ClinkcodeDataSet{
    var code:Int32 = 0
    var loggedInWithInstances:[ClinkcodeInstanceData] = []
    var numHighErrorEncounters:Int = 0
    var allowHighErrorEncounters:Bool = false
    var latestEncounter:Clinkcode?
    var latestInstance:ClinkcodeInstanceData?
    var instances:[ClinkcodeInstanceData] = []
    
    var newTriangulations:[ClinkcodeInstanceData] = []
    var newInstances:[ClinkcodeInstanceData] = []
    
    //requests
    var poseTrackingRequested:Bool = false
    var isMeasuringTagSize:Bool = true
    
    init(_ withCode:Int32){
        code = withCode
    }
    
    func isLoggedIn() -> Bool{
        return loggedInWithInstances.count > 0
    }
    
    func newEncounter(_ encounter:Clinkcode ){
        guard poseTrackingRequested || isMeasuringTagSize else {return}
        guard encounter.hasLowErrorPose() else{ return }
        var isNewInstance = true
        for instance in instances{
            if(instance.isPossibleMatch(encounter)){
                isNewInstance = false
                switch instance.newEncounter(encounter){
                case .TagSizeTriangulated :
                    print("TagSize Triangulated: \(instance.tagSize)")
                    newTriangulations.append(instance)
                default : break
                }
            }
        }
        if(isNewInstance){
            let newInstance = ClinkcodeInstanceData(encounter)
            instances.append(newInstance)
            newInstances.append(newInstance)
        }
    }
    
    func findMatchingInstance(_ forClinkcode:Clinkcode ) -> ClinkcodeInstanceData? {
        for instance in instances{
            if(instance.isPossibleMatch(forClinkcode)){
                return instance
            }
        }
        return nil
    }
    
}

class Clinkcode{
    let clinkDataFrame:ClinkDataFrame
    var topLeft:ExtractedTag
    var topRight:ExtractedTag
    var bottomLeft:ExtractedTag
    var bottomRight:ExtractedTag
    
    var isValid:Bool = false
    var projectionMatrix:[Double] = []
    var code:Int32 = 0
    var centerXY:(x:Double, y:Double) = (-1,-1)
    
    //pose info:
    var poseError:Double? = nil
    var globalQuaternion:[Double]? = nil
    var unitTranslation:[Double]? = nil //multiply by tagSize to get the actual translation
    var tilt:[Double]? = nil //three angles. First is upDownTilt, second is compass angle, third is z rotation when upright
    
    init( cwPair:OppositeTagPair, ccwPair:OppositeTagPair, pixelBufferBaseAddress:UnsafeRawPointer, dataFrame:ClinkDataFrame ){
        clinkDataFrame = dataFrame
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
        projectionMatrix = buildClinkcodeProjectionMatrix(self)
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
            projectionMatrix = buildClinkcodeProjectionMatrix(self)
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
            var lum = readLuminancePixelBuffer(baseAddress: pixelBufferBaseAddress, xy:pt)
            guard lum != nil else{
                return 0
            }
            avgLum += Int(lum!)
            diagonalA.append(Int(lum!))
            pt = projectPoint(m:m , pt: (x:Double(5-i), y: Double(i)))
            lum = readLuminancePixelBuffer(baseAddress: pixelBufferBaseAddress, xy:pt)
            guard lum != nil else{
                return 0
            }
            avgLum += Int(lum!)
            diagonalB.append(Int(lum!))
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
                let lum = readLuminancePixelBuffer(baseAddress: pixelBufferBaseAddress, xy:pt)
                guard lum != nil else{
                    return 0
                }
                if(lum! < avgLum){
                    codeValue = codeValue | (1 << bitIndex)
                }
                bitIndex = bitIndex + 1
            }
        }
        let diagonalIndex = VALID_CLINKCODE_DIAGONALS.index( of: diagonalValue )!
        let hexString = String(codeValue, radix:16)
        let hash = (simpleHash(hexString)+5) % 8
        
        guard hash == diagonalIndex else{
            return 0
        }
        return codeValue
    }
    
    func getCorner(_ index:Int ) -> ExtractedTag {
        switch index{
        case 0: return topLeft
        case 1: return topRight
        case 2: return bottomRight
        default: return bottomLeft
        }
    }
    
    func isColocatedOnScreen(_ clinkcode:Clinkcode ) -> Bool{
        for i in 0...3{
            if( self.getCorner(i).distTo(clinkcode.getCorner(i)) > 9) {
                return false
            }
        }
        return true
    }
    
    func hasLowErrorPose() -> Bool{
        if(poseError == nil){
            calculatePose()
        }
        guard let e = poseError else {return false}
        return e < LOW_ERROR_POSE_THRESHOLD
    }
    
    func isNearCenter() -> Bool{
        let dist = distanceBetween(SCREEN_CENTER, [centerXY.x,centerXY.y,0])
        return dist < CENTER_DIST_THRESHOLD
    }
    
    //Tilt is measured in degrees
    func getTilt() -> [Double]? {
        guard tilt == nil else {return tilt} //cached
        guard let gQuat = getGlobalQuaternion() else { return nil }
        let rotatedZ = applyQuatToVect(gQuat,[0,0,-1])
        let updownPitch = asin(rotatedZ[1]) * 180 / Double.pi
        let compassAngle = -atan2(-rotatedZ[0],-rotatedZ[2]) * 180 / Double.pi
        let rotatedX = applyQuatToVect(gQuat,[1,0,0])
        let zRoll = asin(rotatedX[1]) * 180 / Double.pi
        tilt = [updownPitch,compassAngle,zRoll]
        return tilt
    }
    
    func getGlobalQuaternion() -> [Double]?{
        calculatePose()
        return self.globalQuaternion
    }
    
    func getUnitTranslation() -> [Double]?{
        calculatePose()
        return self.unitTranslation
    }
    
    func calculatePose(){
        guard self.globalQuaternion == nil else {return}
        let poseInfo = estimateClinkcodePose(self)
        self.globalQuaternion = poseInfo["quaternion"]
        self.unitTranslation = poseInfo["translation"]
        self.poseError = poseInfo["error"]?[0]
    }
}


/* ************************** */
/*      HELPER FUNCTIONS      */
/* ************************** */

func flattenMatrix(_ mat4x4:simd_float4x4) -> [Double]{
    var flattened:[Double] = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    for (c, column) in [mat4x4.columns.0, mat4x4.columns.1, mat4x4.columns.2, mat4x4.columns.3].enumerated(){
        for (r, item) in column.enumerated(){
            let i:Int = c*4 + r
            flattened[i] = Double(item)
        }
    }
    return flattened;
}

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

func buildClinkcodeProjectionMatrix(_ clinkcode : Clinkcode ) -> [Double] {
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

func readLuminancePixelBuffer( baseAddress:UnsafeRawPointer, xy:(x:Double, y:Double) ) -> UInt8? {
    let x = Int(round(xy.x))
    let y = Int(round(xy.y))
    guard x >= 0 && x < NUM_PIX_X && y >= 0 && y < NUM_PIX_Y else{
        return nil
    }
    //let offset:Int = (NUM_PIX_X-x)*NUM_PIX_Y + y //Used when buffer is XY flipped
    let offset:Int = y*NUM_PIX_X + x
    let lum:UInt8 = baseAddress.load(fromByteOffset: offset, as: UInt8.self)
    return lum
}

func simpleHash(_ s:String ) -> Int {
    var h = 0;
    for c in s.utf8{
        h = (31*h + Int(c)) & 0xffffffff;
    }
    return h
}

func distanceBetween(_ vectorA:[Double],_ vectorB:[Double] ) -> Double{
    var sum:Double = 0
    for i in 0...2{
        let diff = vectorA[i] - vectorB[i]
        sum += diff*diff
    }
    return sqrt(sum)
}

func vectorLength(_ vector:[Double] ) -> Double{
    return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2])
}

func normalizeVector(_ vector:[Double]) -> [Double]{
    let len = vectorLength(vector)
    return [vector[0]/len, vector[1]/len, vector[2]/len]
}

func crossVectors(_ a:[Double],_ b:[Double]) -> [Double]{
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]
}


func gpsDistanceBetween(_ locationA:[Double],_ locationB:[Double]) -> Double{
    let earthRadius:Double = 6371000 //meters
    let dLat = (locationB[0] - locationA[0])*Double.pi/180
    let dLng = (locationB[1] - locationA[1])*Double.pi/180
    let a = pow(sin(dLat/2),2) + cos(locationA[0]*Double.pi/180) * cos(locationB[0]*Double.pi/180) * pow(sin(dLng/2),2)
    let c = atan2(sqrt(a), sqrt(1-a))
    return earthRadius * c
}

func estimateClinkcodePose(_ clinkcode:Clinkcode) -> Dictionary<String, [Double]>{
    guard let clinkcodePoseFunction = clinkcodePoseFunction else {return [:]}
    let cameraPose = clinkcode.clinkDataFrame.cameraPose
    let focalLength = clinkcode.clinkDataFrame.focalLength
    let corners:[Double] = [clinkcode.topLeft.x - halfPixWidth, clinkcode.topLeft.y - halfPixHeight,
                            clinkcode.topRight.x - halfPixWidth, clinkcode.topRight.y - halfPixHeight,
                            clinkcode.bottomRight.x - halfPixWidth, clinkcode.bottomRight.y - halfPixHeight,
                            clinkcode.bottomLeft.x - halfPixWidth, clinkcode.bottomLeft.y - halfPixHeight]
    
    //Arguments: focalLength, corners, cameraPose
    let result = clinkcodePoseFunction.call(withArguments: [focalLength, corners, cameraPose] )
    let pose = result?.toDictionary() as? Dictionary<String, [Double]> ?? [:]
    //print("quat \(quaternionToEulerDeg(pose["quaternion"]!))")
    //print("trans \(pose["translation"]!)")
    return pose
}

func triangulateTagSize(_ clinkcodeA:Clinkcode,_ clinkcodeB:Clinkcode) -> TriangulatedClinkcodeData? {
    guard let triangulateFunction = triangulatePointsFunction else {return nil}
    let cameraMatrixA = clinkcodeA.clinkDataFrame.cameraPose
    let cameraProjectionA = clinkcodeA.clinkDataFrame.cameraProjection
    let cameraMatrixB = clinkcodeB.clinkDataFrame.cameraPose
    let cameraProjectionB = clinkcodeB.clinkDataFrame.cameraProjection
    let screenPointsA = [clinkcodeA.centerXY.x, clinkcodeA.centerXY.y,
                         clinkcodeA.topLeft.x, clinkcodeA.topLeft.y,
                         clinkcodeA.topRight.x, clinkcodeA.topRight.y,
                         clinkcodeA.bottomRight.x, clinkcodeA.bottomRight.y,
                         clinkcodeA.bottomLeft.x, clinkcodeA.bottomLeft.y]
    let screenPointsB = [clinkcodeB.centerXY.x, clinkcodeB.centerXY.y,
                         clinkcodeB.topLeft.x, clinkcodeB.topLeft.y,
                         clinkcodeB.topRight.x, clinkcodeB.topRight.y,
                         clinkcodeB.bottomRight.x, clinkcodeB.bottomRight.y,
                         clinkcodeB.bottomLeft.x, clinkcodeB.bottomLeft.y]
    
    //Arguments: triangulatePointsFrom2CameraViews( pixW, pixH, cameraMatrixA, cameraProjectionA, screenPointsA, cameraMatrixB, cameraProjectionB, screenPointsB ){
    let result = triangulateFunction.call(withArguments: [NUM_PIX_X, NUM_PIX_Y, cameraMatrixA, cameraProjectionA, screenPointsA, cameraMatrixB, cameraProjectionB, screenPointsB ] )
    guard let triangulationData = result?.toDictionary() as? Dictionary<String, [Double]>,
        let distances = triangulationData["distances"],
        let aPoints = triangulationData["aPoints"],
        let bPoints = triangulationData["bPoints"],
        let translationA = clinkcodeA.getUnitTranslation(),
        let translationB = clinkcodeB.getUnitTranslation()
        else {return nil}
    guard distances[0] > 0 && distances[1] > 0 && distances[2] < 0.01 else
    {
        return nil
        
    }
    
    let center:[Double] = getAverageLocation([Double](aPoints[0...2]), [Double](bPoints[0...2]))
    let topLeft:[Double] = getAverageLocation([Double](aPoints[3...5]), [Double](bPoints[3...5]))
    let topRight:[Double] = getAverageLocation([Double](aPoints[6...8]), [Double](bPoints[6...8]))
    let bottomRight:[Double] = getAverageLocation([Double](aPoints[9...11]), [Double](bPoints[9...11]))
    let bottomLeft:[Double] = getAverageLocation([Double](aPoints[12...14]), [Double](aPoints[12...14]))
    let leftPoint = getAverageLocation(topLeft, bottomLeft)
    let rightPoint = getAverageLocation(topRight, bottomRight)
    let topPoint = getAverageLocation(topLeft, topRight)
    let bottomPoint = getAverageLocation(bottomLeft, bottomRight)
    let topLength = distanceBetween(topLeft, topRight)
    let bottomLength = distanceBetween(bottomLeft, bottomRight)
    let leftLength = distanceBetween(topLeft, bottomLeft)
    let rightLength = distanceBetween(topRight, bottomRight)
    
    let distA = vectorLength(translationA)
    let tSizeA = distances[0] / distA
    let tSizeB = distances[1] / vectorLength(translationB)
    
    let sizeEstimates = [topLength, bottomLength, leftLength, rightLength, tSizeA, tSizeB]
    //let avgSize = (topLength + bottomLength + leftLength + rightLength + tSizeA + tSizeB)/6
    //let avgSize = (topLength + bottomLength + leftLength + rightLength + tSizeA + tSizeB - sizeEstimates.min()!)/5
    let avgSize = (tSizeA + tSizeB)/2
    //let avgSize = max(tSizeA, tSizeB)
    let sizeError = (sizeEstimates.max()! - sizeEstimates.min()!) * 1000 //error in mm
    
    let hAxis = normalizeVector([rightPoint[0]-leftPoint[0], rightPoint[1]-leftPoint[1], rightPoint[2]-leftPoint[2]])
    let vAxis = normalizeVector([topPoint[0]-bottomPoint[0], topPoint[1]-bottomPoint[1], topPoint[2]-bottomPoint[2]])
    let zAxis = crossVectors(vAxis,hAxis);
    
    let tilt = [-asin(zAxis[1]) * 180 / Double.pi,-atan2(zAxis[0],zAxis[2]) * 180 / Double.pi,asin(hAxis[1]) * 180 / Double.pi]
    let avgPoseTilt = getAverageTilt(clinkcodeA.getTilt()!, clinkcodeB.getTilt()!)
    let tiltError = (abs(tilt[0] - avgPoseTilt[0]) + abs(tilt[1] - avgPoseTilt[1]) + abs(tilt[2] - avgPoseTilt[2])) / 10.0
    let poseError = (clinkcodeB.poseError! + clinkcodeB.poseError!) / 6.0
    let totalError = sizeError + tiltError + poseError
    
    return TriangulatedClinkcodeData(frame:clinkcodeB.clinkDataFrame, error: totalError, tagSize: avgSize, centerPoint:center, topLeftPoint: topLeft, topRightPoint: topRight, bottomRightPoint: bottomRight, bottomLeftPoint: bottomLeft, tilt: avgPoseTilt)
}


func getAverageLocation(_ a:[Double],_ b:[Double]) -> [Double]{
    return [(a[0] + b[0])/2,(a[1] + b[1])/2,(a[2] + b[2])/2]
}


func getAngleBetweenQuaternions(_ a:[Double], _ b:[Double] ) -> Double{
    let innerProduct = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
    return acos(2*innerProduct*innerProduct - 1);
}

func applyQuatToVect(_ q:[Double], _ vect:[Double] ) -> [Double]{
    let x = vect[0];
    let y = vect[1];
    let z = vect[2];
    let qx = q[0];
    let qy = q[1];
    let qz = q[2];
    let qw = q[3];
    
    let ix =  qw * x + qy * z - qz * y;
    let iy =  qw * y + qz * x - qx * z;
    let iz =  qw * z + qx * y - qy * x;
    let iw = -qx * x - qy * y - qz * z;
    
    return [ ix * qw + iw * -qx + iy * -qz - iz * -qy,
             iy * qw + iw * -qy + iz * -qx - ix * -qz,
             iz * qw + iw * -qz + ix * -qy - iy * -qx];
}

func quaternionToEulerDeg(_ quat:[Double]) -> [Double]{
    let x = quat[0]
    let y = quat[1]
    let z = quat[2]
    let w = quat[3]
    
    let Rx = atan2(2.0 * (w * x + y * z), 1.0 - (2.0 * (x * x + y * y)))
    let Ry = asin(2.0 * (w * y - z * x))
    let Rz = atan2(2.0 * (w * z + x * y), 1.0 - (2.0  * (y * y + z * z)))
    return [Rx*180/Double.pi, Ry*180/Double.pi, Rz*180/Double.pi]
}

func getQuaternionFromMatrix(_ mat:[Double] ) -> [Double]{
    var quat = [0.0,0.0,0.0,1.0];
    let trace = mat[0] + mat[5] + mat[10];
    var S = 0.0;
    if (trace > 0) {
        S = sqrt(trace + 1.0) * 2;
        quat[3] = 0.25 * S;
        quat[0] = (mat[6] - mat[9]) / S;
        quat[1] = (mat[8] - mat[2]) / S;
        quat[2] = (mat[1] - mat[4]) / S;
    } else if ((mat[0] > mat[5]) && (mat[0] > mat[10])) {
        S = sqrt(1.0 + mat[0] - mat[5] - mat[10]) * 2;
        quat[3] = (mat[6] - mat[9]) / S;
        quat[0] = 0.25 * S;
        quat[1] = (mat[1] + mat[4]) / S;
        quat[2] = (mat[8] + mat[2]) / S;
    } else if (mat[5] > mat[10]) {
        S = sqrt(1.0 + mat[5] - mat[0] - mat[10]) * 2;
        quat[3] = (mat[8] - mat[2]) / S;
        quat[0] = (mat[1] + mat[4]) / S;
        quat[1] = 0.25 * S;
        quat[2] = (mat[6] + mat[9]) / S;
    } else {
        S = sqrt(1.0 + mat[10] - mat[0] - mat[5]) * 2;
        quat[3] = (mat[1] - mat[4]) / S;
        quat[0] = (mat[8] + mat[2]) / S;
        quat[1] = (mat[6] + mat[9]) / S;
        quat[2] = 0.25 * S;
    }
    return quat;
}

func getDegreeDifference(_ deg1:Double,_ deg2:Double ) -> Double{
    let diff = abs(deg1 - deg2)
    return diff < 180 ? diff : 360 - diff
}

func getAverageTilt(_ tiltA:[Double],_ tiltB:[Double]) -> [Double]{
    var avgTilt = [(tiltA[0]+tiltB[0])/2.0, (tiltA[1]+tiltB[1])/2.0, (tiltA[2]+tiltB[2])/2.0]
    if((tiltA[1] > 90 && tiltB[1] < -90) || (tiltA[1] < -90 && tiltB[1] > 90)){
        let c = (tiltA[1] + tiltB[1] + 360)/2.0
        avgTilt[1] = c < 180 ? c : c-360
    }
    return avgTilt
}
