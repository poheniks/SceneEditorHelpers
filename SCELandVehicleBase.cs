using System;
using TaleWorlds.MountAndBlade;
using TaleWorlds.Engine;
using TaleWorlds.Localization;
using TaleWorlds.Library;
using TaleWorlds.Core;
using TaleWorlds.InputSystem;
using TaleWorlds.GauntletUI.PrefabSystem;
using System.Collections.Generic;

namespace SceneEditorExtras
{
    /* NOTES
     ** REQUIREMENTS:
     * 
     *  * Bullets Key:
     *      - : An actionable option/idea
     *      ? : A questionable or difficult option/idea
     *      / : A possible option/idea
     *      * : A note/statement/fact
     * 
     *  * Driving:
     *      ? AI can drive
     *      - Use StandingPoint object to lock driver (agent) in place
     *      ? Dynamically create & modify a path with tangency to handle motion
     *      / Ship-based steering (Held): steering angle is held & does not return to zero
     *      / Quick steering (Direct): steering angle returns to zero after no driver input
     *      * Still need to investigate how to handle terrain impassibles 
     *      
     *  * AI Navigation:
     *      - Write new AI behavior & mission behavior
     *      - Can use StandingPoint parented to vehicle base for fixed AI placement
     *      - Can write something like a TacticalPosition for dynamic AI placement (preferred)
     *  
     *  * Navmesh Handling
     *      - Use logic in SiegeTower class
     * 
     ** PRIORITY:
     *      1: Navmesh Handling
     *      2: Driving
     *      3: AI Navigation
     */

    public class SCE_1LandVehicleBase : UsableMachine
    {
        //force = mass * acceleration
        //KE = 1/2 * mass * velocity^2
        //PE = mass * gravity * height

        private enum WheelSide
        {
            FL = 0,
            FR = 1,
            RL = 2,
            RR = 3
        }

        public float topSpeed = 5f;     //velocity = length/sec
        public float turnRadius = 10f;  //length
        public float steerRate = 1f;    //coefficient and/or non-dimensional
        public float power = 1f;        //force
        public float grip = 0.5f;       //friction coefficient (NOTE: roll resistance coefficient is 0.1*grip)

        public float wheelRate = 10f;    //spring rate: force / length
        public float rideHeight = 1f;   //wheel offset; positive is upwards
        public float suspensionTravel = 2f;     //wheel droop before pulling parent body downwards; positive is downwards

        public bool enableGhost = true;
        public Vec3 debugForce = new Vec3(0, 0, 0);
        public Vec3 debugMoment = new Vec3(0, 0, 0);
        public bool debug1 = false;
        public bool debugLoad = false;

        private GameEntity vehicle;
        private GameEntity ghost;

        private Dictionary<GameEntity, WheelSide> wheels = new Dictionary<GameEntity, WheelSide>();
        private Dictionary<GameEntity, float> wheelAndGroundPos = new Dictionary<GameEntity, float>();
        private Dictionary<WheelSide, Tuple<float, Vec2, int>> wheelSides = new Dictionary<WheelSide, Tuple<float, Vec2, int>>();
        private Dictionary<GameEntity, float> wheelVelocities = new Dictionary<GameEntity, float>();

        private Scene scene;
        private SCEVehiclePhysLib physLib;
        private bool hasDriver = false;
        private bool isStatic = true;

        private float inputLongitude;
        private float inputLatitude;

        private float longitudeVel;     //  unit/sec
        private float analogTurn;       //  non-dimensional

        private Vec3 vehicleNormal;     //  vehicle orientation up vector (z-axis)
        private Vec3 vehicleFwd;        //  vehicle orientation fwd vector

        private float gravity = 9.81f;  // acceleration = length/sec^2

        private Tuple<Vec3, Vec3> acceleration = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));        //translational & rotational accelerations
        private Tuple<Vec3, Vec3> velocity = Tuple.Create(new Vec3(0,0,0), new Vec3(0,0,0));                //translational & rotational velocities

        public override string GetDescriptionText(GameEntity gameEntity = null)
        {
            return new TextObject("LandVehicleBase", null).ToString();
        }

        public override TextObject GetActionTextForStandingPoint(UsableMissionObject usableGameObject)
        {
            return new TextObject("Driver", null);
        }
        //descriptor overrides

        protected override TickRequirement GetTickRequirement()
        {
            return ScriptComponentBehaviour.TickRequirement.Tick;
        }

        protected override void OnEditorInit()
        {
            base.OnEditorInit();
            vehicle = base.GameEntity;
            scene = vehicle.Scene;

            vehicle.BodyFlag = BodyFlags.BodyOwnerEntity;
            physLib = new SCEVehiclePhysLib(vehicle, vehicle.Mass, power, topSpeed, turnRadius, grip, gravity);
        }

        protected override void OnEditorVariableChanged(string variableName)
        {
            base.OnEditorVariableChanged(variableName);

            if (variableName == "enableGhost" && ghost != null)
            {
                physLib = new SCEVehiclePhysLib(ghost, vehicle.Mass, power, topSpeed, turnRadius, grip, gravity);
                velocity = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));

                List<GameEntity> wheelChildren = vehicle.CollectChildrenEntitiesWithTag("wheel");
                if (enableGhost && wheelChildren.Count > 0)
                {
                    velocity = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));
                    foreach (GameEntity child in wheelChildren)
                    {
                        vehicle.RemoveChild(child, true, true, false, 0);
                        ghost.AddChild(child, true);
                    }
                }
                else if (!enableGhost && wheelChildren.Count > 0)
                {
                    velocity = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));
                    ghost.SetGlobalFrame(vehicle.GetGlobalFrame());
                    foreach (GameEntity child in wheelChildren)
                    {
                        ghost.RemoveChild(child, true, true, false, 0);
                        vehicle.AddChild(child, true);
                    }
                }
            }
        }

        protected override void OnEditorTick(float dt)
        {
            base.OnEditorTick(dt);
            List<GameEntity> wheelChildren = vehicle.CollectChildrenEntitiesWithTag("wheel");
            ghost = vehicle.GetFirstChildEntityWithTag("ghost");
            
            if (ghost != null) ghost.BodyFlag = BodyFlags.BodyOwnerEntity;

            if (wheelChildren.Count > 0 && enableGhost) 
            {
                TickExternalForces(ghost, dt);
                TickFrame(ghost, dt);
            }
        }

        protected override void OnInit()
        {
            base.OnInit();
            base.SetScriptComponentToTick(this.GetTickRequirement());

            vehicle = base.GameEntity;
            List<GameEntity> wheels = vehicle.CollectChildrenEntitiesWithTag("wheel");
            ghost = vehicle.GetFirstChildEntityWithTag("ghost");
            if (ghost != null) ghost.BodyFlag = BodyFlags.CommonCollisionExcludeFlags;
            scene = vehicle.Scene;

            if (NavMeshPrefabName.Length > 0) AttachDynamicNavmeshToEntity();
            physLib = new SCEVehiclePhysLib(vehicle, vehicle.Mass, power, topSpeed, turnRadius, grip, gravity);

        }

        protected override void OnTick(float dt)
        {
            base.OnTick(dt);

        }

        private void TickExternalForces(GameEntity vehicle, float dt)
        {

            MatrixFrame frame = vehicle.GetGlobalFrame();
            float mass = vehicle.Mass;
            Vec3 pos = vehicle.GlobalPosition;
            Vec3 centerMass = vehicle.CenterOfMass;

            Vec3 massPos = pos + frame.rotation.TransformToParent(centerMass);
            Vec3 massDir = frame.rotation.TransformToLocal(new Vec3(0, 0, -gravity * mass));
            Tuple<Vec3, Vec3, Vec3, bool> weight = Tuple.Create(massDir, Vec3.Zero, massPos, false);

            Vec3 translationalVelocity = velocity.Item1;
            Tuple<Vec3, Vec3, Vec3, bool> translationalDrag = Tuple.Create(physLib.CalculateDragForces(translationalVelocity), Vec3.Zero, Vec3.Zero, true);

            Vec3 rotationalVelocity = velocity.Item2;
            Tuple<Vec3, Vec3, Vec3, bool> rotationalDrag = Tuple.Create(Vec3.Zero, physLib.CalculateDragForces(rotationalVelocity*25), Vec3.Zero, true);

            Vec3 testForce = frame.rotation.TransformToLocal(debugForce);

            Vec3 testMoment = debugMoment;
            Tuple<Vec3, Vec3, Vec3, bool> testRotation = Tuple.Create(testForce, testMoment, Vec3.Zero, true);

            List<Tuple<Vec3, Vec3, Vec3, bool>> loads = new List<Tuple<Vec3, Vec3, Vec3, bool>>();

            loads.Add(translationalDrag);
            loads.Add(rotationalDrag);
            if (debugLoad) loads.Add(testRotation);
            if (!debugLoad) loads.Add(weight);

            acceleration = physLib.ApplyLoadsAndGetAccelerations(vehicle, loads);

            //SCEMath.DebugMessage("a" + translationalVelocity.ToString());
            //SCEMath.DebugMessage("rotate" + acceleration.Item2.ToString());
        }

        private void TickFrame(GameEntity vehicle, float dt)
        {
            MatrixFrame refFrame = vehicle.GetGlobalFrame();
            MatrixFrame nextFrame = vehicle.GetGlobalFrame();

            Vec3 translations = velocity.Item1;
            Vec3 prevRotations = velocity.Item2;
            Vec3 rotations = velocity.Item2;

            translations += acceleration.Item1 * dt;
            rotations += acceleration.Item2 * dt;
            
            rotations = SCEMath.LimitVectorComponents(rotations, 10000f);
            nextFrame.rotation.RotateAboutSide(rotations.x * dt);
            nextFrame.rotation.RotateAboutForward(rotations.y * dt);
            nextFrame.rotation.RotateAboutUp(rotations.z * dt);

            if (debug1) translations = physLib.AdjustFrameVelocityForRotation(translations, prevRotations, rotations, refFrame, nextFrame);

            translations = SCEMath.LimitVectorComponents(translations, 10000f);
            nextFrame.Strafe(translations.x * dt);
            nextFrame.Advance(translations.y * dt);
            nextFrame.Elevate(translations.z * dt);

            velocity = Tuple.Create(translations, rotations);

            float groundHeight = scene.GetGroundHeightAtPosition(vehicle.GlobalPosition, BodyFlags.BodyOwnerEntity);
            if (nextFrame.origin.z < groundHeight) nextFrame.origin.z = groundHeight;

            vehicle.SetGlobalFrame(nextFrame);
        }


        private void SetWheels(List<GameEntity> wheelChildren)
        {

            wheels.Clear();
            wheelVelocities.Clear();
            foreach(GameEntity wheel in wheelChildren)
            {
                Vec3 wheelPos = wheel.GetFrame().origin;
                wheel.BodyFlag = BodyFlags.BodyOwnerEntity;
                if (wheelPos.y >= 0)
                {
                    if (wheelPos.x <= 0)
                    {
                        wheels.Add(wheel, WheelSide.FL);   //FL
                    }
                    else
                    {
                        wheels.Add(wheel, WheelSide.FR);   //FR
                    }
                }
                else
                {
                    if (wheelPos.x <= 0)
                    {
                        wheels.Add(wheel, WheelSide.RL);   //RL
                    }
                    else
                    {
                        wheels.Add(wheel, WheelSide.RR);   //RR
                    }
                }
                wheelVelocities.Add(wheel, 0f);
            }
        }


        private void HumanDriverTick()
        {
            Vec2 inputVec = PilotAgent.MovementInputVector;
            inputLongitude = inputVec.Y;
            inputLatitude = -inputVec.X; 

        }

    }
}




/*
    public class SCE_1LandVehicleBase : UsableMachine
{
    //force = mass * acceleration
    //KE = 1/2 * mass * velocity^2
    //PE = mass * gravity * height

    public float topSpeed = 5f;     //velocity = length/sec
    public float turnRadius = 10f;  //length
    public float steerRate = 1f;    //coefficient and/or non-dimensional
    public float power = 1f;        //force
    public float grip = 0.5f;       //friction coefficient (NOTE: roll resistance coefficient is 0.1*grip)

    public float wheelRate = 10f;    //spring rate: force / length
    public float rideHeight = 1f;   //wheel offset; positive is upwards
    public float suspensionTravel = 2f;     //wheel droop before pulling parent body downwards; positive is downwards

    public bool enableGhost = true;

    private GameEntity vehicle;
    private GameEntity ghost;

    private Dictionary<GameEntity, WheelSide> wheels = new Dictionary<GameEntity, WheelSide>();
    private Dictionary<GameEntity, float> wheelAndGroundPos = new Dictionary<GameEntity, float>();
    private Dictionary<WheelSide, Tuple<float, Vec2, int>> wheelSides = new Dictionary<WheelSide, Tuple<float, Vec2, int>>();
    private Dictionary<GameEntity, float> wheelVelocities = new Dictionary<GameEntity, float>();

    private Scene scene;
    private SCEVehiclePhysLib physLib;
    private bool hasDriver = false;
    private bool isStatic = true;

    private float inputLongitude;
    private float inputLatitude;

    private float longitudeVel;     //  unit/sec
    private float analogTurn;       //  non-dimensional

    private Vec3 vehicleNormal;     //  vehicle orientation up vector (z-axis)
    private Vec3 vehicleFwd;        //  vehicle orientation fwd vector

    private float gravity = 9.81f;  // acceleration = length/sec^2

    private Tuple<Vec3, Vec3> acceleration = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));        //translational & rotational accelerations
    private Tuple<Vec3, Vec3> velocity = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));                //translational & rotational velocities

    public override string GetDescriptionText(GameEntity gameEntity = null)
    {
        return new TextObject("LandVehicleBase", null).ToString();
    }

    public override TextObject GetActionTextForStandingPoint(UsableMissionObject usableGameObject)
    {
        return new TextObject("Driver", null);
    }
    //descriptor overrides

    protected override TickRequirement GetTickRequirement()
    {
        return ScriptComponentBehaviour.TickRequirement.Tick;
    }

    protected override void OnEditorInit()
    {
        base.OnEditorInit();
        vehicle = base.GameEntity;
        scene = vehicle.Scene;

        vehicle.BodyFlag = BodyFlags.BodyOwnerEntity;
        physLib = new SCEVehiclePhysLib(vehicle, vehicle.Mass, power, topSpeed, turnRadius, grip, gravity);
    }

    protected override void OnEditorVariableChanged(string variableName)
    {
        base.OnEditorVariableChanged(variableName);
        physLib = new SCEVehiclePhysLib(ghost, vehicle.Mass, power, topSpeed, turnRadius, grip, gravity);

        if (variableName == "enableGhost" && ghost != null)
        {
            List<GameEntity> wheelChildren = vehicle.CollectChildrenEntitiesWithTag("wheel");
            if (enableGhost && wheelChildren.Count > 0)
            {
                velocity = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));
                foreach (GameEntity child in wheelChildren)
                {
                    vehicle.RemoveChild(child, true, true, false, 0);
                    ghost.AddChild(child, true);
                }
            }
            else if (!enableGhost && wheelChildren.Count > 0)
            {
                velocity = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));
                ghost.SetGlobalFrame(vehicle.GetGlobalFrame());
                foreach (GameEntity child in wheelChildren)
                {
                    ghost.RemoveChild(child, true, true, false, 0);
                    vehicle.AddChild(child, true);
                }
            }
        }
    }

    protected override void OnEditorTick(float dt)
    {
        base.OnEditorTick(dt);
        List<GameEntity> wheelChildren = vehicle.CollectChildrenEntitiesWithTag("wheel");
        ghost = vehicle.GetFirstChildEntityWithTag("ghost");

        if (ghost != null) ghost.BodyFlag = BodyFlags.BodyOwnerEntity;

        if (wheelChildren.Count > 0)
        {
            SetWheels(wheelChildren);
            UpdateWheelAndGroundPositions();
            CalculateAndSetWheelPositions(dt);
            UpdateAxlePlanes();
            TickPrimaryVerticalForces();

            if (enableGhost) SetGhostFrame(dt);
            else ghost.SetGlobalFrame(vehicle.GetGlobalFrame());

        }
    }

    protected override void OnInit()
    {
        base.OnInit();
        base.SetScriptComponentToTick(this.GetTickRequirement());

        vehicle = base.GameEntity;
        List<GameEntity> wheels = vehicle.CollectChildrenEntitiesWithTag("wheel");
        ghost = vehicle.GetFirstChildEntityWithTag("ghost");
        if (ghost != null) ghost.BodyFlag = BodyFlags.CommonCollisionExcludeFlags;
        scene = vehicle.Scene;

        if (NavMeshPrefabName.Length > 0) AttachDynamicNavmeshToEntity();
        physLib = new SCEVehiclePhysLib(vehicle, vehicle.Mass, power, topSpeed, turnRadius, grip, gravity);

        List<GameEntity> wheelChildren = vehicle.CollectChildrenEntitiesWithTag("wheel");
        if (wheelChildren.Count > 0) SetWheels(wheelChildren);
        if (wheels.Count > 0) UpdateWheelAndGroundPositions();
    }

    protected override void OnTick(float dt)
    {
        base.OnTick(dt);

        if (PilotAgent != null) hasDriver = true;
        else
        {
            hasDriver = false;
            inputLongitude = 0f;
            inputLatitude = 0f;

        }

        if (hasDriver && !PilotAgent.IsAIControlled) HumanDriverTick();

        isStatic = (inputLongitude == 0 && longitudeVel <= 0.002f && longitudeVel >= -0.002f);

        if (!isStatic)
        {
            //steering / lateral motion
            if (inputLatitude != 0) analogTurn += steerRate * inputLatitude * dt;
            else analogTurn = 0f;

            analogTurn = Math.Min(1, Math.Max(-1, analogTurn));
            float turnRate = physLib.CalculateTurnRate(longitudeVel, analogTurn) * dt;

            //longitudinal motion
            longitudeVel += physLib.CalculateLongitudinalAccel(longitudeVel, inputLongitude) * dt;

            //set vehicle frame
            MatrixFrame nextFrame = vehicle.GetGlobalFrame();
            //Mat3 vehicleOrientation = new Mat3(nextFrame.rotation.f, vehicleNormal)

            nextFrame.Advance(longitudeVel * dt);
            nextFrame.Rotate(turnRate, vehicle.GetGlobalFrame().rotation.u);
            vehicle.SetGlobalFrame(nextFrame);

        }
        else
        {
            longitudeVel = 0f;
            analogTurn = 0f;
        }
        if (wheels.Count > 0) UpdateWheelAndGroundPositions();
    }

    private void SetWheels(List<GameEntity> wheelChildren)
    {

        wheels.Clear();
        wheelVelocities.Clear();
        foreach (GameEntity wheel in wheelChildren)
        {
            Vec3 wheelPos = wheel.GetFrame().origin;
            wheel.BodyFlag = BodyFlags.BodyOwnerEntity;
            if (wheelPos.y >= 0)
            {
                if (wheelPos.x <= 0)
                {
                    wheels.Add(wheel, WheelSide.FL);   //FL
                }
                else
                {
                    wheels.Add(wheel, WheelSide.FR);   //FR
                }
            }
            else
            {
                if (wheelPos.x <= 0)
                {
                    wheels.Add(wheel, WheelSide.RL);   //RL
                }
                else
                {
                    wheels.Add(wheel, WheelSide.RR);   //RR
                }
            }
            wheelVelocities.Add(wheel, 0f);
        }
    }

    private void UpdateWheelAndGroundPositions()
    {
        wheelSides = new Dictionary<WheelSide, Tuple<float, Vec2, int>>()
            {
                { WheelSide.FL, new Tuple<float, Vec2 ,int>(0, new Vec2(), 0) },
                { WheelSide.FR, new Tuple<float, Vec2 ,int>(0, new Vec2(), 0) },
                { WheelSide.RL, new Tuple<float, Vec2 ,int>(0, new Vec2(), 0) },
                { WheelSide.RR, new Tuple<float, Vec2 ,int>(0, new Vec2(), 0) }
            };

        //get ground height & set wheel height
        wheelAndGroundPos.Clear();
        foreach (KeyValuePair<GameEntity, WheelSide> wheelKVP in wheels)
        {
            Vec3 wheelPos = wheelKVP.Key.GlobalPosition;
            float groundHeight = scene.GetGroundHeightAtPosition(wheelPos, BodyFlags.BodyOwnerEntity);

            int sideCount = wheelSides[wheelKVP.Value].Item3 + 1;
            float totalHeight = wheelSides[wheelKVP.Value].Item1 + wheelKVP.Key.GlobalPosition.z;
            Vec2 side2DPos = wheelSides[wheelKVP.Value].Item2 + wheelPos.AsVec2;

            wheelSides[wheelKVP.Value] = Tuple.Create(totalHeight, side2DPos, sideCount);
            wheelAndGroundPos.Add(wheelKVP.Key, groundHeight);
        }
    }
    private void CalculateAndSetWheelPositions(float dt)
    {
        suspensionTravel = Math.Max(0, suspensionTravel);   //dummy check for user error

        float wheelMaxTravelZ = vehicle.GlobalPosition.z - suspensionTravel;
        foreach (KeyValuePair<GameEntity, float> wheel in wheelAndGroundPos)
        {
            MatrixFrame wheelFrame = wheel.Key.GetFrame();
            float wheelGlobalHeight = wheel.Key.GlobalPosition.z;
            float wheelLocalHeight = wheelFrame.origin.z;
            int travelDir = Math.Sign(wheelGlobalHeight - wheel.Value);    //positive is downward travel

            bool isWheelAtMinHeight = (wheelLocalHeight < -suspensionTravel);
            bool isWheelAtMaxHeight = (wheelLocalHeight > -rideHeight);

            float tolerance = 0.01f;
            bool isWheelAtLimit = (travelDir > 0 && wheelLocalHeight.ApproximatelyEqualsTo(-suspensionTravel, epsilon: tolerance)) | (travelDir < 0 && wheelLocalHeight.ApproximatelyEqualsTo(-rideHeight, epsilon: tolerance));

            if (!wheelGlobalHeight.ApproximatelyEqualsTo(wheel.Value, epsilon: tolerance) & !isWheelAtLimit)
            {
                wheelVelocities[wheel.Key] += physLib.CalculateWheelVerticalAccel() * dt * -travelDir;
                wheelFrame.Elevate(wheelVelocities[wheel.Key]);

                if (isWheelAtMinHeight) wheelFrame.origin.z = -suspensionTravel;
                if (isWheelAtMaxHeight) wheelFrame.origin.z = -rideHeight;

                //wheel.Key.SetFrame(ref wheelFrame);
            }
            else wheelVelocities[wheel.Key] = 0;

        }
    }

    private void UpdateAxlePlanes()
    {
        //handle multiple wheels per side
        Dictionary<WheelSide, Vec3> wheelPointDict = new Dictionary<WheelSide, Vec3>();
        foreach (KeyValuePair<WheelSide, Tuple<float, Vec2, int>> wheelHeight in wheelSides)
        {
            float totalHeight = wheelSides[wheelHeight.Key].Item1;
            Vec2 centerPos = wheelSides[wheelHeight.Key].Item2;
            int sideCount = wheelSides[wheelHeight.Key].Item3;

            float meanHeight = totalHeight / sideCount;
            Vec3 meanPos = new Vec3((centerPos.x / sideCount), (centerPos.y / sideCount), meanHeight);

            wheelPointDict.Add(wheelHeight.Key, meanPos);
        }

        //handle axle-centerpoints
        Vec3 fCenter = SCEMath.AverageVectors(new List<Vec3>() { wheelPointDict[WheelSide.FL], wheelPointDict[WheelSide.FR] });
        Vec3 rCenter = SCEMath.AverageVectors(new List<Vec3>() { wheelPointDict[WheelSide.RL], wheelPointDict[WheelSide.RR] });

        Vec3 fPlaneNormal = SCEMath.NormalOfPlane(rCenter, wheelPointDict[WheelSide.FL], wheelPointDict[WheelSide.FR]);
        Vec3 rPlaneNormal = SCEMath.NormalOfPlane(fCenter, wheelPointDict[WheelSide.RR], wheelPointDict[WheelSide.RL]);  //need to flip B & C from fPlaneNormal
        Vec3 avgPlaneNormal = SCEMath.AverageVectors(new List<Vec3>() { fPlaneNormal, rPlaneNormal }).NormalizedCopy();

        vehicleNormal = avgPlaneNormal;
        vehicleFwd = (fCenter - rCenter).NormalizedCopy();
    }

    private void TickPrimaryVerticalForces()
    {
        List<Tuple<Vec3, Vec3, bool>> forces = new List<Tuple<Vec3, Vec3, bool>>();

        Vec3 globalCenterMass = SCEMath.LocalArmToGlobal(ghost.GetGlobalFrame().rotation, ghost.CenterOfMass);
        Vec3 globalGravity = SCEMath.LocalArmToGlobal(ghost.GetGlobalFrame().rotation, new Vec3(0, 0, vehicle.Mass * -gravity));
        SCEMath.DebugMessage("GRAV: " + globalGravity.ToString());

        Tuple<Vec3, Vec3, bool> weight = Tuple.Create(globalGravity, globalCenterMass, true);

        /*
        foreach (KeyValuePair<GameEntity, float> wheelKVP in wheelAndGroundPos) //get wheel reaction
        {
            GameEntity wheel = wheelKVP.Key;
            MatrixFrame wheelFrame = wheel.GetFrame();
            float displacement = wheelFrame.origin.z - (-suspensionTravel);
            float force = wheelRate * displacement;
            Vec3 forceVector = ghost.GetGlobalFrame().rotation.u;
            forceVector = forceVector * force;

            Tuple<Vec3, Vec3, bool> inputForce = Tuple.Create(forceVector, wheelFrame.origin, true);
            //forces.Add(inputForce);
        }
        
        forces.Add(weight);
        acceleration = physLib.ApplyForceAndGetAccelerations(forces);

    }

    private void SetGhostFrame(float dt)
    {

        Vec3 translation = velocity.Item1;
        Vec3 rotation = velocity.Item2;
        translation += acceleration.Item1 * dt;
        rotation += acceleration.Item2 * dt;
        velocity = Tuple.Create(translation, rotation);

        MatrixFrame nextFrame = ghost.GetGlobalFrame();
        Vec3 f = nextFrame.rotation.f;
        Vec3 s = nextFrame.rotation.s;
        Vec3 u = nextFrame.rotation.u;

        translation *= dt;
        //nextFrame.origin.x += (translation.x);
        //nextFrame.origin.y += (translation.y);
        //nextFrame.origin.z += (translation.z);
        rotation *= dt;
        nextFrame.Rotate(rotation.x, s);
        nextFrame.Rotate(rotation.y, f);
        nextFrame.Rotate(rotation.z, u);

        float groundHeight = scene.GetGroundHeightAtPosition(ghost.GlobalPosition, BodyFlags.BodyOwnerEntity);
        if (nextFrame.origin.z < groundHeight) nextFrame.origin.z = groundHeight;

        ghost.SetGlobalFrame(nextFrame);

        //SCEMath.DebugMessage("GROUND: " + groundHeight.ToString());
        //SCEMath.DebugMessage("MOVE: " + acceleration.Item1.ToString());
        //SCEMath.DebugMessage("ROT: " + acceleration.Item2.ToString());
    }

    private void HumanDriverTick()
    {
        Vec2 inputVec = PilotAgent.MovementInputVector;
        inputLongitude = inputVec.Y;
        inputLatitude = -inputVec.X;

    }

    private enum WheelSide
    {
        FL = 0,
        FR = 1,
        RL = 2,
        RR = 3
    }
}
*/