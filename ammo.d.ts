declare module Ammo {

    export function destroy(object: any): void;

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
        constructor(t: btTransform);
    }

    export enum CollisionFlags
	{
		CF_STATIC_OBJECT= 1,
		CF_KINEMATIC_OBJECT= 2,
		CF_NO_CONTACT_RESPONSE = 4,
		CF_CUSTOM_MATERIAL_CALLBACK = 8,//this allows per-triangle material (friction/restitution)
		CF_CHARACTER_OBJECT = 16,
		CF_DISABLE_VISUALIZE_OBJECT = 32, //disable debug drawing
		CF_DISABLE_SPU_COLLISION_PROCESSING = 64//disable parallel/SPU processing
	}

    export enum CollisionFilterGroups
	{
        DefaultFilter = 1,
        StaticFilter = 2,
        KinematicFilter = 4,
        DebrisFilter = 8,
        SensorTrigger = 16,
        CharacterFilter = 32,
        AllFilter = -1 //all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger
	}

    export class btCollisionObject {
        getCollisionShape(): btCollisionShape;
        setCollisionShape(collisionShape: btCollisionShape);
        setContactProcessingThreshold(contactProcessingThreshold: number): void;
        //setActivationState(newState: number): void;
        //forceActivationState(newState: number): void;
        //activate(forceActivation: boolean): void;
        isActive(): boolean;
        isKinematicObject(): boolean;
        setRestitution(rest: number): void;
        setFriction(frict: number): void;
        getCollisionFlags(): number;
        setCollisionFlags(flags: number): void;
        getWorldTransform(): btTransform;
        setWorldTransform(worldTrans: btTransform): void;
    }

    export class btCollisionObjectWrapper {

    }

    export class RayResultCallback {
        hasHit(): boolean;
        m_collisionFilterGroup: number;
        m_collisionFilterMask: number;
        m_collisionObject: btCollisionObject;
    }

    export class ClosestRayResultCallback extends RayResultCallback {
        ClosestRayResultCallback(from: btVector3, to: btVector3): void;
        m_rayFromWorld: btVector3;
        m_hitNormalWorld: btVector3;
        m_hitPointWorld: btVector3;
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

    export class btBoxShape extends btCollisionShape {
        constructor(boxHalfExtents: btVector3);
    }

    export class btSphereShape extends btCollisionShape {
        constructor(radius: number);
    }

    export class btCapsuleShape extends btConvexShape {
        constructor(radius: number, height: number);
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
        // applyForce(force: btVector3, rel_pos: btVector3): void;
        applyCentralForce(force: btVector3): void;
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
        // upcast(colObj: btCollisionObject): btRigidBody;
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
        addCollisionObject(collisionObject: btCollisionObject, collisionFilterGroup: number, collisionFilterMask: number): void;
        // getBroadphase(): btBroadphaseInterface;
        // convexSweepTest(castShape: btConvexShape, from: btTransform, to: btTransform, resultCallback: ConvexResultCallback, allowedCcdPenetration: number): void;
        // contactPairTest(colObjA: btCollisionObject, colObjB: btCollisionObject, resultCallback: ContactResultCallback): void;
    }

    export class btDynamicsWorld extends btCollisionWorld {
        addAction(action: btActionInterface): void;
    }

    export class btDiscreteDynamicsWorld extends btDynamicsWorld{
        constructor(dispatcher: btCollisionDispatcher, pairCache: btBroadphaseInterface, constraintSolver: btConstraintSolver, collisionConfiguration: btCollisionConfiguration);
        setGravity(gravity: btVector3): void;
        getGravity(): btVector3;
        addRigidBody(body: btRigidBody): void;
        addRigidBody(body: btRigidBody, group: number, mask: number): void;
        removeRigidBody(body: btRigidBody): void;
        stepSimulation(timeStep: number, maxSubSteps: number);
    }

    export class btActionInterface {

    }

    export class btKinematicCharacterController {
        constructor(ghostObject: btPairCachingGhostObject, convexShape: btConvexShape, stepHeight: number, up: btVector3);
        setUpAxis (axis: number): void;
        setWalkDirection (walkDirection: btVector3): void;
        setVelocityForTimeInterval (velocity: btVector3, timeInterval: number): void;
        warp (origin: btVector3): void;
        preStep (collisionWorld: btCollisionWorld): void;
        playerStep (collisionWorld: btCollisionWorld, dt: number): void;
        setFallSpeed (fallSpeed: number): void;
        setJumpSpeed (jumpSpeed:number): void;
        setMaxJumpHeight (maxJumpHeight: number): void;
        canJump (): boolean;
        jump (): void;
        setGravity (gravity: btVector3): void;
        getGravity (): btVector3;
        setMaxSlope (slopeRadians: number): void;
        getMaxSlope (): number;
        getGhostObject (): btPairCachingGhostObject;
        setUseGhostSweepTest (useGhostObjectSweepTest: boolean): void;
        onGround (): boolean;
    }

    export class btGhostObject extends btCollisionObject {
        getNumOverlappingObjects(): number;
        getOverlappingObject(index: number): btCollisionObject;
    }
    
    export class btPairCachingGhostObject extends btGhostObject {

    }

    export class btGhostPairCallback {

    }
}
