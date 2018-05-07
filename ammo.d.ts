declare module Ammo {

    export function destroy(object: any): void;

    //Given a raw pointer (an integer), returns a wrapped object.
    export function wrapPointer(ptr: number, Class: any): void;

    //Returns a raw pointer.
    export function getPointer(object: any): number;

    //Returns a wrapping of the same pointer but to another class.
    export function castObject(object: any, Class: any): void;

    //Compares two objects’ pointers.
    export function compare(object1: any, object2: any): void;

    export class btVector3 {
        constructor(x: number, y: number, z: number);
        x(): number;
        y(): number;
        z(): number;
        length(): number;
        setX(x: number): void;
        setY(y: number): void;
        setZ(z: number): void;
        setValue(x: number, y: number, z: number): void;
    }

    export class btQuadWord {
        x(): number;
        y(): number;
        z(): number;
        w(): number;
        setX(x: number);
        setY(y: number);
        setZ(z: number);
        setW(w: number);
    }

    export class btQuaternion extends btQuadWord {
        constructor(x: number, y: number, z: number, w: number);
        setValue(x: number, y: number, z: number, w: number): void;
    }

    export class btTransform {
        constructor();
        constructor(q: btQuaternion, v: btVector3);
        setIdentity(): void;
        setOrigin(v: btVector3): void;
        getOrigin(): btVector3;
        setRotation(q: btQuaternion): void;
        getRotation(): btQuaternion;
    }

    export class btMotionState {
        getWorldTransform(worldTrans: btTransform): void;
        setWorldTransform(worldTrans: btTransform): void;
    }

    export class btDefaultMotionState extends btMotionState {
        constructor(startTrans?: btTransform, centerOfMassOffset?: btTransform);
    }

    export class btCollisionObject {
        getCollisionShape(): btCollisionShape;
        setCollisionShape(collisionShape: btCollisionShape);
        setContactProcessingThreshold(contactProcessingThreshold: number): void;
        setActivationState(newState: number): void;
        //forceActivationState(newState: number): void;
        activate(forceActivation: boolean): void;
        isActive(): boolean;
        isKinematicObject(): boolean;
        setRestitution(rest: number): void;
        setFriction(frict: number): void;
        getCollisionFlags(): number;
        setCollisionFlags(flags: number): void;
        getWorldTransform(): btTransform;
        setWorldTransform(worldTrans: btTransform): void;
        getUserPointer(): any;
        setUserPointer(userPointer: AmmoCollider): void;
    }

    export class btCollisionObjectWrapper {

    }

    export class RayResultCallback {
        hasHit(): boolean;

        //m_collisionFilterGroup: number;
        get_m_collisionFilterGroup(): number;
        set_m_collisionFilterGroup(group: number): void;

        //m_collisionFilterMask: number;
        get_m_collisionFilterMask(): number;
        set_m_collisionFilterMask(mask: number): void;

        //m_collisionObject: btCollisionObject;
        get_m_collisionObject(): btCollisionObject;
    }

    export class ClosestRayResultCallback extends RayResultCallback {
        constructor(from: btVector3, to: btVector3);
        //m_rayFromWorld: btVector3;
        //m_rayToWorld: btVector3;

        //m_hitNormalWorld: btVector3;
        get_m_hitNormalWorld(): btVector3;

        //m_hitPointWorld: btVector3;
        get_m_hitPointWorld(): btVector3;
    }

    export class btManifoldPoint {
        getPositionWorldOnA(): btVector3;
        getPositionWorldOnB(): btVector3;

        m_localPointA: btVector3;
        m_localPointB: btVector3;
        m_positionWorldOnB: btVector3;
        m_positionWorldOnA: btVector3;
        m_normalWorldOnB: btVector3;
    }

    export class ConcreteContactResultCallback {
        addSingleResult(cp: btManifoldPoint, colObj0Wrap: btCollisionObjectWrapper, partId0: number, index0: number, colObj1Wrap: btCollisionObjectWrapper, partId1: number, index1: number): void;
    }

    export class btCollisionShape {
        setLocalScaling(scaling: btVector3): void;
        calculateLocalInertia(mass: number, inertia: btVector3): void;
    }

    export class btConvexShape extends btCollisionShape {

    }

    export class btBoxShape extends btConvexShape {
        constructor(boxHalfExtents: btVector3);
    }

    export class btSphereShape extends btConvexShape {
        constructor(radius: number);
    }

    export class btCapsuleShape extends btConvexShape {
        constructor(radius: number, height: number);
    }

    export class btCapsuleShapeX extends btCapsuleShape {
        constructor(radius: number, height: number);
    }

    export class btCapsuleShapeZ extends btCapsuleShape {
        constructor(radius: number, height: number);
    }

    export class btConcaveShape extends btCollisionShape {
    }

    export class btStridingMeshInterface{
    }

    export class btTriangleMesh extends btStridingMeshInterface{
        constructor(use32bitIndices?: boolean, use4componentVertices?: boolean);
        addTriangle(vertex0: btVector3, vertex1: btVector3, vertex2: btVector3, removeDuplicateVertices?: boolean): void;
    }

    export class btTriangleMeshShape extends btConcaveShape {   
    }

    export class btBvhTriangleMeshShape extends btTriangleMeshShape {
        constructor(meshInterface: btStridingMeshInterface, useQuantizedAabbCompression: boolean, buildBvh?: boolean);
    }

    export class btSequentialImpulseConstraintSolver { }

    export class btDefaultCollisionConfiguration {
    }

    export class btPersistentManifold {
        getBody0(): btCollisionObject;
        getBody1(): btCollisionObject;
        getNumContacts(): any;
        getContactPoint(index: number): btManifoldPoint;
    }

    export class btDispatcher {
        getNumManifolds(): number;
        getManifoldByIndexInternal(number): btPersistentManifold;
    }

    export class btCollisionDispatcher extends btDispatcher{
        constructor(conf: btDefaultCollisionConfiguration);
    }

    export class btOverlappingPairCallback {

    }

    export class  btOverlappingPairCache {
        setInternalGhostPairCallback(ghostPairCallback: btOverlappingPairCallback);
    }

    export class btBroadphaseInterface {
        getOverlappingPairCache(): btOverlappingPairCache;
    }

    export class btDbvtBroadphase extends btBroadphaseInterface{

    }

    export class btAxisSweep3 extends btBroadphaseInterface{
        constructor(min: btVector3, max: btVector3);
    }

    export class btCollisionConfiguration {

    }

    export class btRigidBodyConstructionInfo {
        constructor(mass: number, motionState: btMotionState, collisionShape: btCollisionShape, localInertia: btVector3);
        m_linearDamping: number;
        m_angularDamping: number;
        m_friction: number;
        m_rollingFriction: number;
        m_restitution: number;
        m_linearSleepingThreshold: number;
        m_angularSleepingThreshold: number;
        m_additionalDamping: boolean;
        m_additionalDampingFactor: number;
        m_additionalLinearDampingThresholdSqr: number;
        m_additionalAngularDampingThresholdSqr: number;
        m_additionalAngularDampingFactor: number;
    }

    export class btRigidBody extends btCollisionObject {
        constructor(constructionInfo: btRigidBodyConstructionInfo);
        // getCenterOfMassTransform(): btTransform;
        // setCenterOfMassTransform(xform: btTransform): void;
        // setSleepingThresholds(linear: number, angular: number): void;
        // setDamping(lin_damping: number, ang_damping: number): void;
        // setMassProps(mass: number, inertia: btVector3): btTransform;
        // applyTorque(torque: btVector3): void;
        applyForce(force: btVector3, rel_pos: btVector3): void;
        //applyCentralForce(force: btVector3): void;
        // applyTorqueImpulse(torque: btVector3): void;
        // applyImpulse(impulse: btVector3, rel_pos: btVector3): void;
        // applyCentralImpulse(impulse: btVector3): void;
        // updateInertiaTensor(): void;
        // getLinearVelocity(): btVector3;
        // getAngularVelocity(): btVector3;
        // setLinearVelocity(lin_vel: btVector3): void;
        // setAngularVelocity(ang_vel: btVector3): void;
        // setLinearFactor(linearFactor: btVector3): void;
        // setAngularFactor(angularFactor: btVector3): void;
        getMotionState(): btMotionState;
        //upcast(colObj: btCollisionObject): btRigidBody;
    }


    export class btConstraintSolver {

    }

    // export class  btDispatcherInfo {
    //     m_timeStep: number;
    //     m_stepCount: number;
    //     m_dispatchFunc: number;
    //     m_timeOfImpact: number;
    //     m_useContinuous: boolean; 
    //     m_enableSatConvex: boolean;
    //     m_enableSPU: boolean;
    //     m_useEpa: boolean;
    //     m_allowedCcdPenetration: number;
    //     m_useConvexConservativeDistanceUtil: boolean;
    //     m_convexConservativeDistanceThreshold: number;
    // }


    export class btCollisionWorld {
        // getDispatcher(): btDispatcher;
        rayTest(rayFromWorld: btVector3, rayToWorld: btVector3, resultCallback: RayResultCallback): void;
        // getPairCache(): btOverlappingPairCache;
        // getDispatchInfo(): btDispatcherInfo;
        addCollisionObject(collisionObject: btCollisionObject, collisionFilterGroup?: number, collisionFilterMask?: number): void;
        removeCollisionObject(collisionObject: btCollisionObject): void;
        // getBroadphase(): btBroadphaseInterface;
        // convexSweepTest(castShape: btConvexShape, from: btTransform, to: btTransform, resultCallback: ConvexResultCallback, allowedCcdPenetration: number): void;
        // contactPairTest(colObjA: btCollisionObject, colObjB: btCollisionObject, resultCallback: ContactResultCallback): void;
    }

    export class btDynamicsWorld extends btCollisionWorld {
        addAction(action: btActionInterface): void;
        removeAction(action: btActionInterface): void;
    }

    export class btDiscreteDynamicsWorld extends btDynamicsWorld{
        constructor(dispatcher: btCollisionDispatcher, pairCache: btBroadphaseInterface, constraintSolver: btConstraintSolver, collisionConfiguration: btCollisionConfiguration);
        setGravity(gravity: btVector3): void;
        getGravity(): btVector3;
        addRigidBody(body: btRigidBody): void;
        addRigidBody(body: btRigidBody, group: number, mask: number): void;
        removeRigidBody(body: btRigidBody): void;
        stepSimulation(timeStep: number, maxSubSteps?: number, fixedTimeStep?: number);
    }

    export class btActionInterface {

    }

    export class btKinematicCharacterController {
        constructor(ghostObject: btPairCachingGhostObject, convexShape: btConvexShape, stepHeight: number, up?: btVector3);
        //setUp(up: btVector3);
        setWalkDirection (walkDirection: btVector3): void;
        setVelocityForTimeInterval (velocity: btVector3, timeInterval: number): void;
        warp (origin: btVector3): void;
        //reset(collisionWorld: btCollisionWorld): void;
        //preStep (collisionWorld: btCollisionWorld): void;
        //playerStep (collisionWorld: btCollisionWorld, dt: number): void;
        setFallSpeed (fallSpeed: number): void;
        setJumpSpeed (jumpSpeed:number): void;
        //setMaxJumpHeight (maxJumpHeight: number): void;
        //canJump (): boolean;  //canJump内部直接return的onGround
        jump (v?: btVector3): void;
        setGravity (gravity: btVector3): void;
        //getGravity (): btVector3;
        setMaxSlope (slopeRadians: number): void;
        //getMaxSlope (): number;
        setStepHeight (h: number): void;
        //getStepHeight (): number;
        getGhostObject (): btPairCachingGhostObject;
        setUseGhostSweepTest (useGhostObjectSweepTest: boolean): void;
        onGround (): boolean;
    }

    export class btGhostObject extends btCollisionObject {
        getNumOverlappingObjects(): number;
        getOverlappingObject(index: number): btCollisionObject;
        //upcast(colObj: btCollisionObject): btGhostObject;
    }
    
    export class btPairCachingGhostObject extends btGhostObject {

    }

    export class btGhostPairCallback {

    }
}