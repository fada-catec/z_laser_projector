/**
 * Interface description file for the communication between ZLP-Service and a remote client.
 * This Thrift file is processed by the Thrift code generator to produce code for the various
 * target languages.
 */

namespace py zlaser.thrift
namespace cpp zlaser.thrift


struct SemanticVersion_t {
    1: required i32 majorVersion;                           // incompatible API changes
    2: required i32 minorVersion;                           // add backwards-compatible functionality
    3: optional i32 patchVersion = 0;                       // backwards-compatible bug fixes
}

enum ProjectorType_t {
    UNKNOWN,                                                // if projector type cannot be matched to one of the
                                                            // elements below
    A2F500SOM,
    ZLP1,                                                   // aka ZLP-Mini
    ZLP2,
    SIMULATOR,
}

struct TeachTable_t {
    1: required double minX;
    2: required double maxX;
    3: required double minY;
    4: required double maxY;
}

struct ProjectorInfo_t {
    1: required SemanticVersion_t hwRevision;               // on-board revision
    2: required SemanticVersion_t fpgaIp;                   // FPGA IntellectualProperty
    3: required SemanticVersion_t firmware;
    4: required SemanticVersion_t lpcom;                    // software API version
    5: required string revision;                            // VCS revision
    6: required ProjectorType_t kind;
    7: required TeachTable_t teachTable;
}


/** Enums */
/** projection point types */
enum ProjectionPointTypes {
    /** Point of a line (Pen between two of them is down)          */ LINE,
    /** Endpoint of a line (Pen between ENDPT and next LINE is up) */ ENDPT,
}

/** definitions for port numbers to use for connections, server port can be choosed between
    min and max */
enum CommunicationPorts {
    /** minimal usable portnumber for server */ SERVER_MIN = 9000,
    /** maximal usable portnumber for server */ SERVER_MAX = 10000,
    /** default portnumber for server        */ SERVER_DEFAULT = 9090,
}

/** defines set types for clip groups, to decide visiblity of elements/segments */
enum ClipSetTypes {
    /** Element is visible if all Clipping results
    left it visible                               */ Intersection,
    /** Element is visible if at least one Clipping
    result left it visible                        */ Union
}

/** Types (Id's), that can be part of the geometrytree */
enum GeoTreeElemTypes {
    /** unspecified or unknown element type                */ UNSPECIFIED_BASE   = 0,

    /** Bit to identify enum as a Mask for Class of
        Elements                                           */ MASK_BIT           = 0x10000,
    /** Mask for the Element-Type bits                     */ TYPE_MASK          = 0x0FF00,
    /** for use in GetElementIds to get all elementtypes   */ ALL_TYPE           = 0x1FF00,
    /** Type for all Groups                                */ GROUP_TYPE         = 0x10100,
    /** A Group for all types of elements, should be in
        ALL, GROUP                                         */ EL_GROUP           = 0x00100,
    /** Flag for all 3D-Elements (includes projectable
        and clip-types)                                    */ EL_3D_TYPE         = 0x10200,
    /** Group for 3D-Element, should be in
        ALL, GROUP, EL_3D                                  */ GROUP_3D           = 0x00300,
    /** original Data from a 3D-Pointscan                  */ SCANDATA           = 0x00200,
    /** genuine pointcloud                                 */ POINTCLOUD         = 0x00201,
    /** triangulated scandata, or surface data             */ TRIANGLEMESH       = 0x00203,
    /** Flag for all projectable Elements                  */ PROJECTION_EL_TYPE = 0x10400,
    /** Polyline                                           */ POLYLINE           = 0x00400,
    /** CircleSegment                                      */ CIRCLESEGMENT      = 0x00401,
    /** TextElement                                        */ TEXTELEMENT        = 0x00402,
    /** OvalSegment                                        */ OVALSEGMENT        = 0x00403,
    /** Flag for all Clipping-Type-Elements                */ CLIP_EL_TYPE       = 0x10800,
    /** Clipplane                                          */ CLIPPLANE          = 0x00800,
    /** Clipping group                                     */ CLIP_GROUP         = 0x00900,
    /** Referenc-Type-Elements                             */ REF_EL_TYPE        = 0x11000,
    /** Referenceobject                                    */ REFOBJECT          = 0x01000,
    /** Flag for Driftcompensation-Type-Elements           */ DC_EL_TYPE         = 0x12000,
    /** Driftcompensationobject                            */ DC_OBJECT          = 0x02000,
    /** ClipRect                                           */ CLIP_RECT          = 0x04000,
}

/** Different levels for Logging (lowercase names fixes msvc 11 bug) */
enum LogLevels {
    /** debugging info              */ debug = 1,
    /** standard level for messages */ info = 2,
    /** if something happens        */ warning = 4,
    /** if an error occurs          */ error = 8,
    /** if the error is critical    */ critical = 16,
    /** mask for all levels         */ all = 31,
}

/** FunctionModuel States, Values as they were returned by the function:
FunctionModuleGetState */
enum FunctionModuleStates {
    /** initial state of the service / initializing */ UNITIALIZED,
    /** Waiting for start of a function module      */ IDLE,
    /** Function Module is running                  */ RUNNING,
    /** Stop was requested for a running
        functionModule                              */ STOP_REQUESTED,
    /** Error occurred, but module can be started
        again, string can be retrieved by
        FunctionModuleGetErrorString                */ RECOVERABLE_ERROR,
    /** serious Error occurred, string can be
        retrieved by FunctionModuleGetErrorString   */ CRITICAL_ERROR
}

/** States of reflexion point search */
enum PointSearchStates {
    /** initial state of point search / initializing */ INIT,
    /** ready for start of point search              */ IDLE,
    /** Error occurred, invalid configuration        */ INVALID_CFG
    /** starting point search                        */ START_SEARCH,
    /** point search in progress                     */ RUNNING,
    /** point found                                  */ POINT_FOUND,
    /** no point found                               */ NO_POINT_FOUND
}

/** Service States, Values as they were returned by the functions:
ServiceWaitStateChange and ServiceGetState */
enum ServiceStates {
    /** initial state of the service / initializing */ UNITIALIZED,
    /** Hardware is starting                        */ START_HARDWARE,
    /** Waiting for start of a function module      */ IDLE,
    /** Function Module is running                  */ RUN,
    /** Service is going down                       */ SHUTDOWN_HARDWARE,
    /** serious Error occurred, string can be
        retrieved by ServiceGetLastException        */ CRITICAL,
}

/** Command codes for remotecontrol functions */
enum RcCommandCodes {
    KEY_FOCUS_IN     =  1,
    KEY_ON_OFF       =  3,
    KEY_MODE         =  4,
    KEY_PLUS         =  5,
    KEY_FOCUS_OUT    =  6,
    KEY_MINUS        = 10,
    KEY_UP_LEFT      = 17,
    KEY_UP           = 18,
    KEY_UP_RIGHT     = 19,
    KEY_LEFT         = 22,
    KEY_HOME         = 23,
    KEY_RIGHT        = 24,
    KEY_DOWN_LEFT    = 27,
    KEY_DOWN         = 28,
    KEY_DOWN_RIGHT   = 29,
    KEY_ROTATE_LEFT  = 32,
    KEY_ROTATE_RIGHT = 34,
    KEY_SEARCH       = 38,
    KEY_P1_P9        = 46,
    KEY_P2_P10       = 47,
    KEY_P3_P11       = 48,
    KEY_P4_P12       = 49,
    KEY_ALL          = 50,
    KEY_P5_P13       = 51,
    KEY_P6_P14       = 52,
    KEY_P7_P15       = 53,
    KEY_P8_P16       = 54,
    KEY_2ND          = 55,
}

/** Parameters for Transformation function */
enum ReferencePlane {
    /* Plane for Relect method */
    PLANE_YZ = 1,
    PLANE_XZ = 2,
}

/** Structs */

struct Vector2D {
    1: required double x;
    2: required double y;
}

struct Vector3D {
    1: required double x;
    2: required double y;
    3: required double z;
}

typedef Vector3D Point3D
typedef Vector2D Point2D

/** RGB - color - value */
struct RGBValue {
    /** Red-Value   */ 1: required i16 r;
    /** Green-Value */ 2: required i16 g;
    /** Blue-Value  */ 3: required i16 b;
}

/** Matrix, Access data: data.at(row).at(col) */
struct Matrix4x4 {
    /** Matrix Data */ 1: required list<list<double>> data,
}

/** Base structure für GeoTree-Elements */
struct GeoTreeElement {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation      */ 2: required bool   activated = true,
}

/** Base Clippingelement for multiprojectorsystems */
struct ClipElement {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation        */ 2: required bool              activated = true,
    /** map of projector associated value to
        decide whether to project                 */ 3: required map<string, bool> projectorMap,
    /** name of corresponding coordinatesystem    */ 4: required string            coordinateSystem
}

/** Clippingplane for multiprojectorsystems
Map contains all associated projectors as Key(string) and a Flag as Value(boolean)
The Flag describes, if the facing (0) or the averted side (1) is projected by the
associated projector */
struct ClipPlane {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation        */ 2: required bool              activated = true,
    /** map of projector associated value to
        decide whether to project                 */ 3: required map<string, bool> projectorMap,
    /** name of corresponding coordinatesystem    */ 4: required string            coordinateSystem,
    /** point of plane in Point-Normal-Form       */ 5: required Point3D           point,
    /** normal of plane in Point-Normal-Form      */ 6: required Point3D           normal
}

/** Rectangle for clipping the projection data */
struct ClipRect {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation                           */ 2: required bool              activated = true,
    /** Flag for visualisation                        */ 3: required bool              showOutline = false,
    /** Flag for activation of clipping               */ 4: required bool              clippingActivated = false,
    /** Position of lower left corner of the element  */ 5: required Point3D           position,
    /** height, width of element                      */ 6: required Vector2D          size,
    /** name of corresponding coordinatesystem        */ 7: required string            coordinateSystem
}

/** Base structure for 3d-Elements */
struct Element3d {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation        */ 2: required bool      activated = true,
    /** Usertransformation                        */ 3: required Matrix4x4 userTrans,
}

/** Base structure for Projection Elements */
struct ProjectionElement {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation        */ 2: required bool         activated = true,
    /** Usertransformation                        */ 3: required Matrix4x4    userTrans,
    /** Pen number, color to use                  */ 4: required i16          pen,
    /** List of coordinate systems                */ 5: required list<string> coordinateSystemList,
    /** List of projectors                        */ 6: required list<string> projectorIDList,
    /** Flag to enable reflection detection       */ 7: optional bool         detectReflection = false,
}

/** Transmission of polylines */
struct Polyline {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */ 2: required bool         activated = true,
    /** Usertransformation                     */ 3: required Matrix4x4    userTrans,
    /** Pen number, color to use               */ 4: required i16          pen,
    /** List of coordinate systems             */ 5: required list<string> coordinateSystemList,
    /** List of projectors                     */ 6: required list<string> projectorIDList,
    /** Flag to enable reflection detection    */ 7: optional bool         detectReflection = false,
    /** List of Polylines                      */ 8: required list<list<Point3D>> polylineList,
}

/** Struct for CircleSegments, a circle will be drawn at the x/y - Plane
if leaving startAngle / endAngle untouched, the segment will be a complete circle */
struct CircleSegment {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */  2: required bool         activated = true,
    /** Usertransformation                     */  3: required Matrix4x4    userTrans,
    /** Pen number, color to use               */  4: required i16          pen,
    /** List of coordinate systems             */  5: required list<string> coordinateSystemList,
    /** List of projectors                     */  6: required list<string> projectorIDList,
    /** Flag to enable reflection detection    */  7: optional bool         detectReflection = false,
    /** Centerpoint                            */  8: required Point3D      center,
    /** Radius                                 */  9: required double       radius,
    /** Difference in mm between
        circle and chord                       */ 10: optional double       chordHeight,
    /** StartAngle in Degree                   */ 11: optional double       startAngle,
    /** EndAngle in Degree                     */ 12: optional double       endAngle,
    /** Normal Vector of drawing
           plane, to draw circle               */ 13: optional Vector3D     normal,
}

struct TextElement {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */  2: required bool         activated = true,
    /** Usertransformation                     */  3: required Matrix4x4    userTrans,
    /** Pen number, color to use               */  4: required i16          pen,
    /** List of coordinate systems             */  5: required list<string> coordinateSystemList,
    /** List of projectors                     */  6: required list<string> projectorIDList,
    /** Flag to enable reflection detection    */  7: optional bool         detectReflection = false,
    /** Position                               */  8: required Point3D      position,
    /** Text to render                         */  9: required string       text,
    /** Height of font                         */ 10: required double       height,
    /** Name or path of font                   */ 11: optional string       font,
    /** Normal Vector of drawing
        plane, to render text                  */ 12: optional Vector3D     normal,
    /** Rotation angle in Degree               */ 13: optional double       angle,
    /** Space between characters               */ 14: optional double       charSpacing,
    /** Space between lines                    */ 15: optional double       lineSpacing,
}

/** Struct for OvalSegments, an oval will be drawn at the x/y - Plane
if leaving startAngle / endAngle untouched and Width=Height, the segment will be a complete circle */
struct OvalSegment {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */  2: required bool         activated = true,
    /** Usertransformation                     */  3: required Matrix4x4    userTrans,
    /** Pen number, color to use               */  4: required i16          pen,
    /** List of coordinate systems             */  5: required list<string> coordinateSystemList,
    /** List of projectors                     */  6: required list<string> projectorIDList,
    /** Flag to enable reflection detection    */  7: optional bool         detectReflection = false,
    /** Centerpoint                            */  8: required Point3D      center,
    /** Width                                  */  9: required double       width,
    /** Height                                 */ 10: optional double       height,
    /** Angle                                  */ 11: optional double       angle,
    /** Difference in mm between
        oval and chord                         */ 12: optional double       chordHeight,
    /** StartAngle in Degree                   */ 13: optional double       startAngle,
    /** EndAngle in Degree                     */ 14: optional double       endAngle,
    /** Normal Vector of drawing
        plane, to draw oval                    */ 15: optional Vector3D     normal,
}

struct Referencepoint {
    /** Identifier */                               1: required string name,
    /** Use point for projector setup */            2: required bool activated = true,
    /** 3D world coordinate */                      3: required Point3D refPoint,
    /** Intital 2D trace point in mm */             4: required Point2D tracePoint,
    /** Width and height of search area in mm */    5: required Point2D crossSize,
    /** Distance to projector in mm */              6: required double distance,
}

/** Struct for ReferenceObject, used for assignment of coordinatesystem to a projector via a
transformation which is calculated from correlated points (3d and tracepoints)
ReferenceObject is deactivated by default to prevent usage if transformation matrix is not calcualted (fieldTransMat = unit matrix)*/
struct Referenceobject {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */ 2: required bool                 activated = false,
    /** Calculated fieldtransformation from
        3D-Setup                               */ 3: required Matrix4x4            fieldTransMat,
    /** List of initial ReferencePoints        */ 4: required list<Referencepoint> refPointList,
    /** ID of the projector that uses the
        transformation                         */ 5: required string               projectorID,
    /** Name of the corresponding
        coordinatesystem                       */ 6: required string               coordinateSystem,
}

/** Struct for DriftCompensationPoint */
struct DriftCompensationPoint {
    /** Identifier                               */ 1: required string  name,
    /** Flag, if DC-Point should be used for
        calculation                              */ 2: required bool    activated = true,
    /** Flag, if DC-Point should be used as
        reserve for other point of same name     */ 3: required bool    isReserve,
    /** unit vector to Reflexpoint, found at
        initialisation of Driftcompensation      */ 4: required Point3D traceVector,
    /** CrossSize / Searcharea at distance in mm */ 5: required double  crossSize,
    /** Distance to Projector-Origin in mm
        distanceToOrigin * traceVector gives the
        3d coordinate of Reflectionpoint in
        projectorcoordinatesystem                */ 6: required double  distanceToOrigin,
    /** Weight (cnt of usage in Calculation)     */ 7: optional i16     weight,
}

/** Struct for DriftCompensationObject */
struct DriftCompensationObject {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation */ 2: required bool                 activated = true,
    /** Calculated fieldtransformation
        from Drift                         */ 3: required Matrix4x4            compensationTransMat,
    /** List of DriftCompensationPoints    */ 4: required list<DriftCompensationPoint> dcPointList,
    /** ID of the projector that uses the
        transformation                     */ 5: required string               projectorID,
    /** Switches to restrict compensation
        for translation along axes         */ 6: required list<bool>           fixedTranslation,
    /** Switches to restrict compensation
        for rotation around axes           */ 7: required list<bool>           fixedRotation
}

/** Groupelements (group is defining a branch in the geotree */
struct GeoTreeGroup {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */ 2: required bool   activated = true,
}

/** 3d-Groupelements, is basically a Group with additional Properties of a 3d-Element */
struct Group3d {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */ 2: required bool      activated = true,
    /** Usertransformation                     */ 3: required Matrix4x4 userTrans,
}

/** Groupelement for Clipping, should be extended by Clip-Element properties */
struct ClipGroup {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */ 2: required bool              activated = true,
    /** map of projector associated value to
        decide whether to project              */ 3: required map<string, bool> projectorMap,
    /** name of corresponding coordinatesystem */ 4: required string            coordinateSystem,
    /** Type of set to decide visibility of
        group elements                         */ 5: required ClipSetTypes      setType
}

/** 3D Measure Points to build a pointcloud, for keeping internal relations from the fact that the
points were generated by a frame-based Laserscan frameNo and imgRow are included to recreate a
triangulated surface */
struct ScanPoint3D {
    /** Pointcoordinate x                                         */ 1: required double x,
    /** Pointcoordinate y                                         */ 2: required double y,
    /** Pointcoordinate z                                         */ 3: required double z,
    /** confidence from 0...1, created from Grayvalue und Distance
        of crossrays, distance to laserplane ...                  */ 4: required double confidence,
    /** Number of the image, where the point was calculated from  */ 5: required i16 frameNo,
    /** RowNumber within the image where the point was calculated,
        together with frameNo, this describes the 3d-Points in
        2D-Grid - coordinates, this accelerates a wrapping of the
        points into  a surface - mesh for  example in Geomagic a
        lot.                                                      */ 6: required i16 imgRow
}

struct ScanData {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */ 2: required bool              activated = true,
    /** Usertransformation                     */ 3: required Matrix4x4         userTrans,
    /** List of ScanPoint3D                    */ 4: required list<ScanPoint3D> points,
    /** List of Pointcolors                    */ 5: optional list<RGBValue>    pointColors,
}

struct PointCloud {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */ 2: required bool           activated = true,
    /** Usertransformation                     */ 3: required Matrix4x4      userTrans,
    /** List of Vector3D                       */ 4: required list<Vector3D> points,
    /** List of Pointcolors                    */ 5: optional list<RGBValue> pointColors,
    /** List of Pointnormals                   */ 6: optional list<Vector3D> pointNormals,
}

struct Triangle {
    /** index of first  Point */ 1: required i32 idx0,
    /** index of second Point */ 2: required i32 idx1,
    /** index of third  Point */ 3: required i32 idx3,
}

/** Transmission of triangulated Scandata */
struct TriangleMesh {
    /** complete element path (group/group/element) */ 1: required string name,
    /** Flag for activation, visualisation     */  2: required bool           activated = true,
    /** Usertransformation                     */  3: required Matrix4x4      userTrans,
    /** List of Point3D                        */  4: required list<Vector3D> points,
    /** List of Triangles                      */  5: required list<Triangle> triangles,
    /** Inside Color / color of Base is
        outside                                */  6: required RGBValue       insideColor,
    /** List of Trianglenormals                */  7: optional list<Vector3D> triangleNormals,
    /** List of Pointnormals                   */  8: optional list<Vector3D> pointNormals,
    /** List of Pointcolors                    */  9: optional list<RGBValue> pointColors,
    /** List of TriangleColors                 */ 10: optional list<RGBValue> triangleColors,
}

/** Identificator of Geometrydata (e.g. Groups, Polylines, CircleSegments, OvalSegments)
modificationTimeStamp is used for synchronisation purposes. Each time the element is modified,
the modificationTimeStamp is updated. That's way the client can check, whether he has to reload the
data  */
struct GeoTreeElemId {
    /** name e.g. Group1/Group2/Polyline1    */ 1: required string           name,
    /** \see GeoTreeElemTypes                */ 2: required GeoTreeElemTypes elemType,
    /** for synchronization with client side
         geoTree representation              */ 3: required i64              modificationTimeStamp,
}

/** Transmission of Log-Entries */
struct LogEntry {
    /** timestamp of entry in milliseconds
        since 1970,01,01                 */ 1: required i64    timeStamp,
    /** level of entry                   */ 2: required i32    level,
    /** message                          */ 3: required string text,
}
/** Id of command for getting list of remotecontrol-commands */
struct RcCommandId {
    /** Mode of RC.                */ 1: required i16  mode,
    /** Command of the IR frame.   */ 2: required i16  cmd,
    /** Second Key activated by RC */ 3: optional bool second = false,
}

/** Exceptions */

/** File is not readable because ... */
exception CantReadFile {
    /** filename */ 1: string fileName,
    /** reason   */ 2: string why
}
/** File is not writable because ... */
exception CantWriteFile {
    /** filename */ 1: string fileName,
    /** reason   */ 2: string why
}
/** File is not deletable because ... */
exception CantRemoveFile {
    /** filename */ 1: string fileName,
    /** reason   */ 2: string why
}

/** GeoTree related exceptions */

/** Coordinate system is not renamable because ...  */
exception CantRenameCoordinatesystem {
    /** coordinate system */ 1: string which,
    /** reason            */ 2: string why
}
/** Geotree elment does not exist */
exception ElementDoesNotExist {
    /** name of element */ 1: string which,
    /** parentname      */ 2: string parent,
    /** childname       */ 3: string child,
}
/** Geotree element is from wrong type */
exception ElementHasWrongType {
    /** name of element */ 1: string which,
    /** parentname      */ 2: string parent,
    /** childname       */ 3: string child,
}
/** an elementpath for Geotree is not in relative format */
exception InvalidRelativePath {
    /** name of element */ 1: string which,
    /** parentname      */ 2: string parent,
    /** childname       */ 3: string child,
}
/** an element cant be set as child because of some reason */
exception CantSetChild {
    /** reason     */ 1: string why,
    /** parentname */ 2: string parent,
    /** childname  */ 3: string child,
}
/** an element already exists */
exception ElementAlreadyExists {
    /** reason     */ 1: string why,
    /** parentname */ 2: string parent,
    /** childname  */ 3: string child,
}

/** ElementTransformation related exceptions */
/** the TransFormParam parameter has an unsupported value */
exception InvalidTransformParam {
    /** name of element*/ 1: string which,
}

/** functionmodule related exceptions */

/** Class of functionmodule was not registered */
exception FunctionModuleClassNotRegistered {
    /** name of class */ 1: string which
}
/** Class of functionmodule was not licensed */
exception FunctionModuleClassNotLicensed {
    /** name of class */ 1: string which
}
/** Branch for properties of functionmodule is already used by another one */
exception FunctionModulePropertyBranchAlreadyInUse {
    /** name of branch   */ 1: string branchName,
    /** name of instance */ 2: string usedByClass
}
/** Functionmodule is already running */
exception FunctionModuleIsAlreadyRunning {
    /** name of functionmodule instance */ 1: string fModUID
}
/** Functionmodule cant run from within this state */
exception FunctionModuleCantRunFromThisState {
    /** name of functionmodule instance */ 1: string fModUID
}
/** Functionmodule has run in a not recoverable errror */
exception FunctionModuleRunFailedCritical {
    /** name of functionmodule instance */ 1: string fModUID
    /** description of RecoverableError */ 2: string description
}
/** Functionmodule has run in an recoverable errror */
exception FunctionModuleRunFailedRecoverable {
    /** name of functionmodule instance */ 1: string fModUID,
    /** description of RecoverableError */ 2: string description
}
/** Class of functionmodule can not deleted */
exception FunctionModuleCantDeleteModule {
    /** name of instance and why */ 1: string whichAndWhy
}
/** Property with name does not exist */
exception PropertyNotFound {  /** name of property */ 1: string propName }

/** Stream/File of Properties can't be read correctly */
exception PropertyReadError { /** reason */ 1: string why }

/** a Property change callback has thrown an exception */
exception PropertyChangeFailed {
    /** name of property        */ 1: string propName,
    /** name of cmd exception   */ 2: string exceptionClass,
    /** detail of cmd exception */ 3: string exceptionDetail
}
/** a Property can't be set, because of limitations by type or options */
exception PropertySetError {
    /** name of property        */ 1: string propName,
    /** detail of cmd exception */ 2: string exceptionDetail
}
/** Projection coordinate is invalid */
exception InvalidProjectionCoordinate {
    /** reason            */ 1: string  why,
    /** coord             */ 2: Point3D coordinate
    /** line idx          */ 3: i32     lineIdx,
    /** name of element   */ 4: string  elementName,
    /** name of projector */ 5: string  projectorName,
    /** name of coordsys  */ 6: string  coordSysName,
}
/** Service cant communicate with projector */
exception ErrorInProjectorCommunication {
    /** reason */ 1: string  why,
}
/** Projection was somehow limited (clipping or missed projector ...) */
exception LimitedProjection {
    /** reason */ 1: string  why,
}
/** Property not registered yet for access notification of property */
exception PropertyNotRegisteredForAccessNotification {
    /** Name of Property */ 1: string propName,
}

/** Client not registered yet for access notification */
exception ClientNotRegisteredForPropertyAccessNotification {
    /** ID of Client who wants access notification */ 1: string clientId,
    /** Name of Property                           */ 2: string propName,
}
/** functionmodule not registered yet for use */
exception FunctionModuleNotExistent {
    /** Id of functionmodule */ 1: string fModUID,
}
/** Client not registered yet for accessing functionodule */
exception ClientNotRegisteredForFunctionModule {
    /** ID of Client who wants access notification */ 1: string clientId,
    /** Id of functionmodule                       */ 2: string fModUID,
}
/** Invalid parameters for remoteControl command */
exception RemoteControlInvalidParameters {
    /** Which one was wrong, why */ 1: string whichAndWhy,
}
/** an internal process is in state CRITCAL_ERROR */
exception ProcessInCriticalErrorState {
    /** Which one was wrong */ 1: string which,
    /** Description         */ 2: string description
}
/** a python error occurred in some function */
exception PythonError {
    /** Message of that error */ 1: string errorMessage
}
/** locking failed for service while calling interface functionality */
exception ServiceLockTimeout {
    /** Message of that error */ 1: string errorMessage
}

/** interface functions to communicate with the service */
service ServiceInterface {
    ProjectorInfo_t getProjectorInfo(1: string identification)

    /** Opens the specified file and reads the configuration.
    Throws exceptionif the file could'nt load successful. */
    void ReadConfig(/** Name of the config file at the service's
                        data directory. */ 1: string filename)
    throws (1:CantReadFile ea, 2:PropertyReadError eb,
            3:ProcessInCriticalErrorState ec, 4:ServiceLockTimeout ed)

    /** Writes the configuration to the specified file.
    Throws CantWriteFile if file cannot be written. */
    void WriteConfig(/** Name of the config file at the service's data
                         directory. */ 1: string filename)
    throws (1:CantWriteFile ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec)

    /** Reset Service - Configuration
    Reset the configuration, clears all GeoTree Data and closes all projector connections */
    void ResetConfig()
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Write a message to servicelog
    Write a m with specifies loglevel to servicelog */
    void LogMessage(
        /** level of the message, see LogLevels */ 1: LogLevels level,
        /** message                             */ 2: string Message)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Get the last n messages from servicelog
    Returns the list of Logentries */
    list<LogEntry> GetLogMessages(
        /** cnt of messages to get          */ 1: i16 numMessages,
        /** log types to get, see LogLevels */ 2: i32 levelMask)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Get the messages from servicelog since time in timeStamp
    Returns the list of Logentries */
    list<LogEntry> GetLogMessagesSince(
        /** timepoint since when messages to get */ 1: i64 timeStamp
        /** log types to get, see LogLevels      */ 2: i32 levelMask)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Get the actual Systemtime of the Service, e.g. for generating valid
        timestamps to retrieve messages */
    i64 GetTimeStamp()
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Get list of all defined Coordinatesystems.
    This function returns a full list of all defined Coordinatesystems that are saved in the
    service.
    Returns a list of all Coordinatesystems */
    list<string> GetCoordinatesystemList()
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Rename a Coordinatesystem.
    Throws CantRenameCoordinatesystem - if the specified old name of Coordinatesystem cannot be
    found, or the new name already exists */
    void RenameCoordinatesystem(
        /** old name of Coordinatesystem */ 1: string oldname,
        /** new name of Coordinatesystem */ 2: string newname)
    throws (1:CantRenameCoordinatesystem ea,
            2:ProcessInCriticalErrorState eb,
            3:ServiceLockTimeout ec)

    /** Get full list of names and types of all elements in the geometrytree.
    This function returns a full list of all GeometryElements that are saved in the service.
    Each GeometryElement contains a name and a type and the last modificationtime. To access
    the data of a specific Element - e.g. a PolylineElement: call TGetPolyLine(nameofElement)
    Returns a list of all GeoTreeElem. */
    list<GeoTreeElemId> GetGeoTreeIds(
        /** Name of branch, for which the elements are requested, "" or "/" for the whole tree */
        1:string branchName = "",
        /** Type of element to get ids for, ALL to get all element td's */
        2:GeoTreeElemTypes typeOfElement = GeoTreeElemTypes.ALL_TYPE)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec)

    /** Remove a GeoTreeElem
    Removes an existing GeoTreeElem and all of its children.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found */
    void RemoveGeoTreeElem(
        /** Name of the GeoTreeElem, use "" to remove all elements  */
        1: string name = "")
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:InvalidRelativePath ed)

    /** Check if a GeoTreeElem exist */
    bool GeoTreeElemExist(/** Name of the GeoTreeElem */ 1: string name)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec)

    /** Rename a GeoTreeElem, moves it inside the geotree, all missing path branches will be created
        from type GeoTreeGroup, if other group types are wished, one has to create them before
        renaming */
    void RenameGeoTreeElem(
        /** pathname of the existing GeoTreeElem */ 1: string oldPath,
        /** The new pathname                     */ 2: string newPath)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:InvalidRelativePath ed, 5:ElementHasWrongType ee)

    /** Get a GeoTreeElement specified by the name
    Get single GeoTreeElement using name from GeoTree.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found. This function is
    useful to get access to the superclass of all Elementtypes.*/
    GeoTreeElement GetGeoTreeElement(/** name of element to get */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Modify a GeoTreeElement.
    If a GeoTreeElement with the name already exists, the existing Projectionelement will
    be modified. This function is useful to change properties of elements which are derived
    from zGeoTreeElement.
    If there is no GeoTreeElement with the given namen, an exception will be thrown. */
    void UpdateGeoTreeElement(/** Data of GeoTreeElement */ 1: GeoTreeElement el)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:InvalidRelativePath ed, 5:ElementHasWrongType ee)

    /** Get a ProjectionElement specified by the name
    Get singleProjectionElement using name from Geometrytree.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found. This function is
    useful to get access to the superclass of Polyline, CircleSegment, OvalSegment..., properties
    Returns the specified ProjectionElement.*/
    ProjectionElement GetProjectionElement(/** name of element to get */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Modify a ProjectionElement.
    If a Projectionelement with the name already exists, the existing Projectionelement will
    be modified. This function is useful to change properties of elements which are derived
    from zProjectionElement.
    If there is no Projectionelement with the given namen, an exception will be thrown. */
    void UpdateProjectionElement(/** Data of ProjectionElement */ 1: ProjectionElement el)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:InvalidRelativePath ed, 5:ElementHasWrongType ee)

    /** Get a 3D-Element specified by the name
    Get single Element3d using name from Geometrytree.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found. This function is
    useful to get access to the superclass of Projekction-Elements and 3D-Elements, properties
    Returns the specified Element3d.*/
    Element3d GetElement3d(/** name of element to get */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Modify a 3d-Element.
    If a 3d-Element with the name already exists, the existing 3D-Element will
    be modified. This function is useful to change properties of elements which are derived
    from z3dElement.
    If there is no 3d-Element with the given namen, an exception will be thrown. */
    void UpdateElement3d(/** Data of 3d-Element */ 1: Element3d el)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:InvalidRelativePath ed, 5:ElementHasWrongType ee)

    /** Get a PolylineElement specified by the name
    Get single PolylineElement using name from Geometrytree.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified PolylineElement.*/
    Polyline GetPolyLine(
        /** name of element to get              */ 1: string name,
        /** set this flag to true if you want to
            get no data, to reduce overhead     */ 2: bool   onlyAttributes = false)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / modify a PolylineElement.
    If a Polylineelement with the name already exists, the existing Polylineelement will
    be modified. To modify attributes of an existing element you can pass empty point parameter.
    If there is no Polylineelement with the given namen, a new one will be created. */
    void SetPolyLine(/** Data of Polyline */ 1: Polyline poly)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a CircleSegmentElement specified by the name
    Get single CircleSegmentElement using name from Geometrytree.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified CircleSegmentElement.*/
    CircleSegment GetCircleSegment(/** Name of the CircleSegment */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Get a TextElement specified by the name
    Get single TextElement using name from Geometrytree.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified TextElement.*/
    TextElement GetTextElement(/** Name of the TextElement */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Get an OvalSegmentElement specified by the name
    Get single OvalSegmentElement using name from Geometrytree.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified OvalSegmentElement.*/
    OvalSegment GetOvalSegment(/** Name of the OvalSegment */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Get the LineList of a ProjectionElement specified by the name
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the LineList (PointLists) of specified ProjectionElement.*/
    list<list<Point3D>> GetLineList(/** Name of the ProjectionElement */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:InvalidRelativePath ed)

    /** Set / Modify a CircleSegmentElement
    If a CircleSegmentelement with the name already exists, the existing
    CircleSegmentelement will be  modified.
    If there is no CircleSegmentelement with the given namen, a new one will be created. */
    void SetCircleSegment(/** CircleSegmentelement */ 1: CircleSegment circleSeg)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Set / Modify a text element
    If a text element with the name already exists, the existing
    text element will be  modified.
    If there is no text element with the given namen, a new one will be created. */
    void SetTextElement(1: TextElement textElement)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Set / Modify an OvalSegmentElement
    If a OvalSegmentelement with the name already exists, the existing
    OvalSegmentelement will be  modified.
    If there is no OvalSegmentelement with the given namen, a new one will be created. */
    void SetOvalSegment(/** OvalSegmentelement */ 1: OvalSegment ovalSeg)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a GroupElement specified by the name.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified GroupElement. */
    GeoTreeGroup GetGroup(/** Name of the GroupElement */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / Modify a GroupElement
    If a GroupElement with the name already exists, the existing
    GroupElement will be  modified.  If there is no GroupElement with the given name,
    a new one will be created. */
    void SetGroup(/** Name of the GroupElement */ 1: GeoTreeGroup group)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a Group3dElement specified by the name.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified GroupElement. */
    Group3d GetGroup3d(/** Name of the GroupElement */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / Modify a Group3dElement
    If a Group3d with the name already exists, the existing
    GroupElement will be  modified.  If there is no GroupElement with the given name,
    a new one will be created. */
    void SetGroup3d(/** Name of the GroupElement */ 1: Group3d group)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a ClipGroupElement specified by the name.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified GroupElement. */
    ClipGroup GetClipGroup(/** Name of the GroupElement */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / Modify a  ClipGroupElement
    If a  ClipGroup with the name already exists, the existing
    GroupElement will be  modified.  If there is no GroupElement with the given name,
    a new one will be created. */
    void SetClipGroup(/** Name of the GroupElement */ 1:  ClipGroup group)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a ClipRect specified by the name.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified Element. */
    ClipRect GetClipRect(/** Name of the GroupElement */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / Modify a  ClipRect
    If a  ClipRect with the name already exists, the existing
    GroupElement will be  modified.  If there is no ClipRect with the given name,
    a new one will be created. */
    void SetClipRect(/** Name of the GroupElement */ 1:  ClipRect rect)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a Referenceobject specified by the name.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified Referenceobject. */
    Referenceobject GetReferenceobject(/** Name of the Referenceobject */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / Modify a Referenceobject
    If a Referenceobject with the name already exists, the existing Referenceobject will
    be  modified. If there is no Referenceobject with the given namen, a new one will be
    created. */
    void SetReferenceobject(/** Name of Referenceobject */ 1: Referenceobject refObject)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a DriftCompensationObject specified by the name.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified DriftCompensationObject. */
    DriftCompensationObject GetDriftCompensationObject(/** Name of the DC-Object */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / Modify a DriftCompensationObject
    If a DriftCompensationObject with the name already exists, the existing Referenceobject will
    be  modified. If there is no DriftCompensationObject with the given namen, a new one will be
    created. */
    void SetDriftCompensationObject(/** Name of DC-Object */ 1: DriftCompensationObject dcObject)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Creates polylines from a referenceobject to show referencepoint positions.
    For each referencepoint a cross is created. All cross-polylines are put in a group
    named [referenceobjectname]_poly. Via parameter showTracePoints it can be choosen to
    create polylines for the tracepoints in projector_fcw coordinate system or in referenced
    user-coordinte system.
    Returns the name of the created polylinegroup */
    string CreatePolylinesFromRefObject(
        /** name of the referenceobject       */ 1: string name,
        /** shows tracepoint coordinates
        instead of referencepoint coordinates */ 2: bool showTracePoints)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed, 5:ElementDoesNotExist ee)

    /** Get a ClipPlane specified by the name
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified ClipPlane. */
    ClipPlane GetClipPlane(/** Name of the ClipPlane */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / Modify a ClipPlane
    If a ClipPlane with the name already exists, the existing ClipPlane will be  modified.
    If there is no ClipPlane with the given namen, a new one will be created. */
    void SetClipPlane(/** Name of the ClipPlane */ 1: ClipPlane clipPlane)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a 3D-ScanDataElement specified by the name
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified Element. */
    ScanData GetScanData(
        /** name of Element to get                   */ 1: string name,
        /** idx of start element to get data while
            system is scanning if is != 0            */ 2: i32 startIdx)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / modify a ScanDataElement
    If a Element with the name already exists, the existing Element will be modified.*/
    void SetScanData(/** ScanData */ 1: ScanData scanData)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a 3D-PointcloudElement specified by the name
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified Element. */
    PointCloud GetPointCloud (/** name of Element to get */ 1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / modify a PointCloud Element
    If a Element with the name already exists, the existing
    Element will be modified. */
    void SetPointCloud(/** Pointcloud */ 1: PointCloud pointcloud)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Get a 3D-TriangleMeshElement specified by the nameble Elementnames.
    Throws ElementDoesNotExist if the specified GeoTreeElem cannot be found.
    Returns the specified Element. */
    TriangleMesh GetTriangleMesh (
        /** name of Element to get                   */ 1: string name,
        /** idx of start element to get data while
            system is scanning if is != 0            */ 2: i32 startIdx)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Set / modify a TriangleMesh Element
    If a Element with the name already exists, the existing
    Element will be modified. */
    void SetTriangleMesh(/** TriangleMesh */ 1: TriangleMesh triangleMesh)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb, 3:InvalidRelativePath ec,
            4:ElementHasWrongType ed)

    /** Creates a functionmodule from wished classtype, the service can be extended via plugins by
    so called functionmodules, they all follow the same scheme in creating, destroying, running and
    parameterhandling. The interface ensures that there exists only one instance of one class
    if there exists an instance of a classtype with the same branchName FunctionModuleCreate returns
    the name of the already existing functionmodule. Clients who call this function successful will be
    registered to receive functionmodule statechange events.
    Throws FunctionModuleClassNotRegistered if the name is not an existing class.
    Returns unique name of the created functionmodule, to have a identifier for the instance
    to use in the other module functions like Run/Stop */
    string FunctionModuleCreate(
        /** Name of the functionmodule type to create   */ 1:string type,
        /** Branchname to put the modules Properties in */ 2:string branchName)
    throws (1:ProcessInCriticalErrorState ea,
            2:FunctionModuleClassNotRegistered eb,
            3:FunctionModulePropertyBranchAlreadyInUse ec,
            4:FunctionModuleClassNotLicensed ed,
            5:ServiceLockTimeout ee)

    /** registers a client for access to a functionmodule by UID, UID is somehow transferred
        to the client. UID can be used to control functionmodule. A client can unregister itself
        by FunctionModuleRelease */
    void FunctionModuleRegisterClient(
        /** UID of the functionmodule object  */ 1:string fModUID)
    throws (1:ProcessInCriticalErrorState ea,
            2:FunctionModuleNotExistent eb,
            3:ServiceLockTimeout ec)

    /** Releases a created function module instance, if client is the last accessor, function module
        will be deleted */
    void FunctionModuleRelease(
        /** The name from the create call,
        to identify the functionmodule instance */ 1:string functionModuleUID)
    throws (1:ProcessInCriticalErrorState ea,
            2:FunctionModuleNotExistent eb,
            3:ClientNotRegisteredForFunctionModule ec,
            4:FunctionModuleCantDeleteModule ed,
            5:ServiceLockTimeout ee)

    /** Calls the run method of the function module. There are 2 kinds of function modules, one
    that performs some calculation in the Run call and returns true and the other starts some self
    running process and returns immediatly with false. The second one keeps in RunningState after
    returning from the Run call (runs in the background). By asking for the state via
    FunctionModuleGetState one can find when the function module has finished. */
    bool FunctionModuleRun(
        /** function module instance UID */ 1:string functionModuleUID)
    throws (1:FunctionModuleNotExistent ea,
            2:ClientNotRegisteredForFunctionModule eb,
            3:FunctionModuleIsAlreadyRunning ec,
            4:FunctionModuleCantRunFromThisState ed,
            5:FunctionModuleRunFailedCritical ee,
            6:FunctionModuleRunFailedRecoverable ef,
            7:ErrorInProjectorCommunication eg,
            8:ProcessInCriticalErrorState eh,
            9:ServiceLockTimeout ei)

    /** Stops an at background running function module. Returns true if the stop has performed
    immediately, false if one has to wait for statchange to other than RUNNING */
    bool FunctionModuleStop(
        /** function module instance UID */ 1:string functionModuleUID)
    throws (1:FunctionModuleNotExistent ea,
            2:ClientNotRegisteredForFunctionModule eb,
            3:ProcessInCriticalErrorState ec,
            4:ServiceLockTimeout ed)

    /** Sets properties of functionmodules */
    void FunctionModuleSetProperty(
        /** function module instance UID              */ 1:string functionModuleUID,
        /** the propertyname/path which should be set */ 2:string propertyName,
        /** the value to set to the property          */ 3:string propertyValue)
    throws (1:PropertyNotFound ea,
            2:FunctionModuleNotExistent eb,
            3:ClientNotRegisteredForFunctionModule ec,
            4:ProcessInCriticalErrorState ed,
            5:ServiceLockTimeout ee,
            6:PropertyChangeFailed ef,
            7:PropertySetError eg)

    /** Get propertyvalues of functionmodules
    Returns propertyvalue in a convertible string format */
    string FunctionModuleGetProperty(
        /** function module instance UID              */ 1:string functionModuleUID,
        /** name/path of property which should be get */ 2:string propertyName)
    throws (1:PropertyNotFound ea,
            2:FunctionModuleNotExistent eb,
            3:ClientNotRegisteredForFunctionModule ec,
            4:ProcessInCriticalErrorState ed,
            5:ServiceLockTimeout ee)

    /** Get options of property of functionmodules following Options can be retrieved
    (depending on type) "type" Valid Types are int, double, string, bool "min", "max",
    "step" for ranges of numeric types "default" the default value
    Returns propertyvalue in a convertible string format */
    string FunctionModuleGetPropertyOption(
        /** function module instance UID              */ 1:string functionModuleUID,
        /** name/path of property which should be get */ 2:string propertyName,
        /** name/path of option which should be get   */ 3:string optionName)
    throws (1:PropertyNotFound ea,
            2:FunctionModuleNotExistent eb,
            3:ClientNotRegisteredForFunctionModule ec,
            4:ProcessInCriticalErrorState ed,
            5:ServiceLockTimeout ee)


    /** Get a list of all created Function modules for a client can be used to clear any previous
    created function modules Returns strings of all running function modules */
    list<string> FunctionModuleGetInstances()
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Getter which retrieves the remaining runtime of the FunctionModule with the given name in
     seconds. Returns the remaining runtime in seconds, the implementation of the function module
     should set this time by itself. */
    double FunctionModuleGetRemainingRunTime(
        /** function module instance UID */ 1:string functionModuleUID)
    throws (1:FunctionModuleNotExistent ea,
            2:ClientNotRegisteredForFunctionModule eb,
            3:ProcessInCriticalErrorState ec,
            4:ServiceLockTimeout ed)

    /** Getter for an string if some exception occurred while running the module in Background
    Returns string with the error description if some error occurs after Run else a string of
    length 0 */
    string FunctionModuleGetRunExceptionString(
        /** function module instance UID */ 1:string functionModuleUID)
    throws (1:FunctionModuleNotExistent ea,
            2:ClientNotRegisteredForFunctionModule eb,
            3:ProcessInCriticalErrorState ec,
            4:ServiceLockTimeout ed)

    /** Getter for actual FunctionModule state
    Returns the actual functionModule state, \see FunctionModuleStates */
    FunctionModuleStates FunctionModuleGetState(
        /** function module instance UID */ 1:string functionModuleUID)
    throws (1:FunctionModuleNotExistent ea,
            2:ClientNotRegisteredForFunctionModule eb,
            3:ProcessInCriticalErrorState ec,
            4:ServiceLockTimeout ed)

    /** Setter for mode of a specific remotecontrol, Mode 0, the servicemode is only accessible
    via the remotecontrol itself and controls the projector directly.
    Modes 1 to 3 are factory defined modes:
    Mode1 - Projection control
    Mode2 - Adjustment control
    Mode3 - InfoScreen control
    Modes greater 3 are for user defined modes. They can be provided by python files. */
    void RemoteControlSetMode(
        /** Adress of RemoteControl (1...64)            */ 1:i16 addr,
        /** Mode to set for this remotecontrol (1...16) */ 2:i16 mode)
    throws (1:RemoteControlInvalidParameters ea,
            2:ProcessInCriticalErrorState eb,
            3:ServiceLockTimeout ec)

    /** Getter for the current mode of a specific remotecontrol */
    i16 RemoteControlGetMode(
        /** Adress of RemoteControl (1...64) */ 1:i16 addr)
    throws(1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Setter for a list of active remotecontrol addresses, for this service, means that only from
    this remotecontrols commands will be treated. By default all remotecontrols are active */
    void RemoteControlSetActiveAddrs(
        /* list of RC-Addresses to activate */ 1:list<i16> addrs)
    throws (1:RemoteControlInvalidParameters ea,
            2:ProcessInCriticalErrorState eb,
            3:ServiceLockTimeout ec)

    /** Getter for active remotecontrol addresses, for this service.
    Returns a list of addresses */
    list<i16> RemoteControlGetActiveAddrs()
    throws(1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Setter for id of an assigned function-module to a specific remotecontrol. By this, one can
    control different functionmodules via different remotecontrols, which can run in the same mode.
    */
    void RemoteControlSetActiveFunctionModuleUID(
        /* address of remotecontrol */ 1:i16    addr,
        /* UID of functionModule    */ 2:string modUID)
    throws (1:RemoteControlInvalidParameters ea,
            2:ProcessInCriticalErrorState eb,
            3:ServiceLockTimeout ec)

    /** Getter for UID of an assigned function-module to a remotecontrol */
    string RemoteControlGetActiveFunctionModuleUID(
        /* RC-Address */ 1:i16 addr)
    throws (1:RemoteControlInvalidParameters ea,
            2:ProcessInCriticalErrorState eb,
            3:ServiceLockTimeout ec)

    /** Trigger a remotecontrol-frame, for testing and simulation. */
    void RemoteControlTriggerFrame(
        /** address of RC-device   */ 1:i16            adr,
        /** key-command            */ 2:RcCommandCodes cmd,
        /** toggle function active */ 3:bool           toggle = false,
        /** Projector ID           */ 4:i16            projector = 0,
        /** Timestamp              */ 5:i32            timestamp = 0)
    throws (1:RemoteControlInvalidParameters ea,
            2:ProcessInCriticalErrorState eb,
            3:ServiceLockTimeout ec)

    /** Projectors are selected by remotecontrol via an index from 1 to 16, with this function for
    each projector serial number can be selected an index. */
    void RemoteControlSetProjectorIdxForSerial(
        /** Projector IDX    */ 1:i16 projectorIdx,
        /** Projector Serial */ 2:string projectorSerial = "")
    throws (1:RemoteControlInvalidParameters ea,
            2:ProcessInCriticalErrorState eb,
            3:ServiceLockTimeout ec)

    /** Simulate condition that a reflection elements detection changed. */
    void triggerReflectionStateChanged(
         /** name of the element which should be triggered      */ 1: string elementName,
         /** True iff a reflection is detected; False otherwise */ 2: bool state)
        throws (1:ProcessInCriticalErrorState ea,
                2:ServiceLockTimeout eb)

    /** Get a Propertyvalue from the service
    Get Propertyvaluestring for the specified property name.
    Always use fully qualified name: e.g. Group1 or Group3/Group4
    Returns the value for the specified Propery name */
    string GetProperty(/** name/path of property which should be get */ 1: string name)
    throws (1:PropertyNotFound ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec)

    /** Set a Propertyvalue to the service
    Set Propertystring value to the property specified by name */
    void SetProperty(
        /** name/path of property which should be set */ 1: string name,
        /** the value to set to the property          */ 2: string value)
    throws (1:PropertyNotFound     ea,
            2:PropertyChangeFailed eb,
            3:PropertySetError ec,
            4:ProcessInCriticalErrorState ed,
            5:ServiceLockTimeout ee)

    /** Get a list of children of the current property
    Returns a list of children names of the current property name, without "options".
    List is empty if no child exists. */
    list<string> GetPropChildren(
        /** name/path of property which children should be get */ 1: string name)
    throws (1:PropertyNotFound ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec)

    /** Registers client to get signaled if a property value changes */
    void RegisterForChangedProperty(
        /** name/path of property which changes should be signaled */ 1: string name)
    throws (1:PropertyNotFound ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec)

    /** Remove a registered client to get signaled if a property value changes */
    void UnRegisterForChangedProperty(
        /** name/path of property which changes should be signaled */ 1: string name)
    throws (1:PropertyNotFound ea,
            2:PropertyNotRegisteredForAccessNotification eb,
            3:ClientNotRegisteredForPropertyAccessNotification ec,
            4:ProcessInCriticalErrorState ed,
            5:ServiceLockTimeout ee)

    /** Trigger Projection of all activated projectionselements in GeoTree */
    void TriggerProjection()
    throws (1:InvalidProjectionCoordinate ea,
            2:ErrorInProjectorCommunication eb,
            3:ProcessInCriticalErrorState ec,
            4:ServiceLockTimeout ed,
            5:LimitedProjection ee)

    /** Creates a channel from service to client, to use for propagating service events back to the
    client so no polling is needed */
    void ConnectClientEventChannel(
        /** Port at client to connect */ 1:i32 portAtClient)
    throws(1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Disconnects the eventchannel to the client */
    void DisconnectClientEventChannel()
    throws(1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Update projector list and check validness of current license.
     *
     * @returns true iff the current license is valid; false otherwise.
     */
    bool CheckLicense()
    throws(1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb);

    /** Load a new license file.
     *
     * @param[in] filename basename of the licensefile to load; must be located in the data directory
     * @return true iff the current license is valid; false otherwise.
     * @note
     * The loading of the license may be successful and the function still return `false`, i.e.
     * when loading a projector license while no projector is attached.
     */
    bool LoadLicense(1:string filename)
        throws(1:ProcessInCriticalErrorState ea,
               2:ServiceLockTimeout eb);


    /** Transfers data to a new created remote file at the service data directory.
    Used when the service is running at another PC.
    Throws CantWriteFile. */
    void TransferDataToFile(
        /** file data                           */ 1: binary data,
        /** filename at the PC where the
            service is running.                 */ 2: string remoteFilename,
        /** overwrite an existing file can be
            overwritten, else an exception is
            generated                           */  3: bool overwrite = false)
    throws (1:CantWriteFile ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec)

    /** Removes a file at the service filesystem */
    void RemoveFile(
        /** remotePath path to file to delete at the PC where the
            service is running                  */ 1: string remotePath)
    throws (1:CantRemoveFile ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec)

    /** Set color map for pens */
    void SetPenColorMap(/** colorMap mapping of rgb color to pen
                            number */ 1:map<i16,RGBValue> colorMap)
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Get color map for pens
        Returns mapping of rgb color to pen number */
    map<i16, RGBValue> GetPenColorMap()
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Get version number of current project.
        Format x.x.x (MajorRelease.MinorRelease.MaintenanceRelease) */
    string GetVersion()
    throws (1:ProcessInCriticalErrorState ea, 2:ServiceLockTimeout eb)

    /** Moves a projection element by an offset */
    void Translate(/** name of element to move */ 1: string name,
                   /** offset in x direction */   2: double dx,
                   /** offset in y direction */   3: double dy,
                   /** offset in z direction */   4: double dz)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Rotates a projection element around its center or reference point */
    void Rotate(1: string name, 2: double xRotationDegree, 3: double yRotationDegree, 4: double zRotationDegree)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Rotates a projection element around the given point */
    void RotateAroundReference(1: string name, 2: double xRotationDegree, 3: double yRotationDegree, 4: double zRotationDegree, 5: Point3D rotationCenter)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Scales a projection element. Its center or reference point stays fixed. */
    void Scale(                        1: string name,
               /** Scale factor > 0 */ 2: double scaleFactor)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee, 6:InvalidTransformParam ef)

    /** Scales a projection element with respect to a given fixed reference point. */
    void ScaleWithReference(                        1: string name,
                            /** Scale factor > 0 */ 2: double scaleFactor,
                                                    3: Point3D referencePoint)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee, 6:InvalidTransformParam ef)

    /** Reflects a projection element on the specified plane */
    void Reflect(1: string name, 2: ReferencePlane reflectionPlane)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee, 6:InvalidTransformParam ef)

    /** Reflects a projection element on the specified plane at the given reference point */
    void ReflectWithReference(1: string name, 2: ReferencePlane plane, 3: Point3D referencePoint)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee, 6:InvalidTransformParam ef)

    /** Resets the user transformation of a projection element */
    void ResetTransformation(1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)

    /** Applies the user transformation of a projection element permanently. Superior group transformations are not
    taken into account. The user transformation is cleared afterwards. The function can not be applied to a group. */
    void ApplyTransformation(1: string name)
    throws (1:ElementDoesNotExist ea, 2:ProcessInCriticalErrorState eb, 3:ServiceLockTimeout ec,
            4:ElementHasWrongType ed, 5:InvalidRelativePath ee)
}

/** Types for GeoTreeChanged messages */
enum GeoTreeChangeTypes {
    /** Placeholder to define                      */ NO_CHANGE,
    /** barker bit for changes at element element  */ EL_CHANGED_BIT      = 0x10,
    /** attributes of an element changed (no data) */ EL_ATTRIBUTES_CHANGED,
    /** data of an element changed, no attributes  */ EL_DATA_CHANGED,
    /** element created                            */ EL_CREATED,
    /** element removed                            */ EL_REMOVED,
    /** marker bit for changed colormap            */ COLORMAP_CHANGED     = 0x20,
    /** marker bit for changed coordsys            */ COORDSYS_CHANGED     = 0x40,
    /** marker bit for whole tree changes          */ TREE_CHANGED_BIT     = 0x80,
    /** on tree create                             */ TREE_CREATED,
    /** on tree object removed                     */ TREE_REMOVED,
    /** on restored tree                           */ TREE_RESTORED,
    /** on cleared tree data                       */ TREE_CLEARED
}

/** Interface for the back-channel Service -> Client, to send events from service to client */
service ClientEventChannel {
     /** Send path and value of property that was changed. */
    void PropertyChanged(
        /** full path of property that was changed */ 1: string name,
        /** value of property                      */ 2: string value)

     /** Send path and flag for element that was changed. */
    void GeoTreeChanged(
        /** integer value with flags of type GeoTreeChangedFlags */ 1: i32    changedFlags,
        /** identification of changed element                    */ 2: GeoTreeElemId element)

    /** Send changes of Service State */
    void ServiceStateChanged(
        /** old state before change */ 1: ServiceStates oldState,
        /** state after change      */ 2: ServiceStates newState)

    /** Send RemoteControl Frame-received Notification */
    void RemoteControlFrameReceived(
        /** address of RC-device   */ 1:i16            adr,
        /** key-command            */ 2:RcCommandCodes cmd,
        /** toggle function active */ 3:bool           toggle,
        /** Projector ID           */ 4:i16            projector,
        /** Timestamp              */ 5:i32            timestamp)

    /** Send changes of FunctionModule State */
    void FunctionModuleStateChanged(
        /** UID of the funtion module */ 1: string               functionModuleUID,
        /** old state before change   */ 2: FunctionModuleStates oldState,
        /** state after change        */ 3: FunctionModuleStates newState)

    /** Triggered when reflection detection is enabled and the reflection state of an element changes. */
    void onReflectionStateChanged(
         /** name of the element that changed state              */ 1: string elementName,
         /** True iff a reflection was detected; False otherwise */ 2: bool state)

}
