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

     */

    public class SCE_ObjectBase : UsableMachine
    {

        public float contactFriction = 0.5f;       //friction coefficient to terrain or other contacting object (NOTE: roll resistance coefficient is 0.1*grip)
        public float dragCoefficient = 0.1f;

        public bool enableGhost = false;
        public bool debugApplyTransDrag = false;
        public bool debugApplyRotDrag = false;
        public bool debugAdjFrameVel = false;

        public bool debugDisableVerticalTranslation = false;
        public bool applyDebugLoads = false;
        public Vec3 debugForce = new Vec3(0, 0, 0);
        public Vec3 debugMoment = new Vec3(0, 0, 0);

        

        private GameEntity physObject;
        private GameEntity ghost;

        private Scene scene;
        private SCEObjectPhysLib physLib;

        private float gravity = 9.81f;  // acceleration = length/sec^2

        private Tuple<Vec3, Vec3> acceleration = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));        //translational & rotational accelerations
        private Tuple<Vec3, Vec3> velocity = Tuple.Create(new Vec3(0,0,0), new Vec3(0,0,0));                //translational & rotational velocities

        public override string GetDescriptionText(GameEntity gameEntity = null)
        {
            return new TextObject("Object Base", null).ToString();
        }

        public override TextObject GetActionTextForStandingPoint(UsableMissionObject usableGameObject)
        {
            return new TextObject("N/A", null);
        }
        //descriptor overrides

        protected override TickRequirement GetTickRequirement()
        {
            return ScriptComponentBehaviour.TickRequirement.Tick;
        }

        protected override void OnEditorInit()
        {
            base.OnEditorInit();
            physObject = base.GameEntity;
            scene = physObject.Scene;

            physObject.BodyFlag = BodyFlags.BodyOwnerEntity;
            SetupPhysLib(physObject);
        }

        protected override void OnEditorVariableChanged(string variableName)
        {
            base.OnEditorVariableChanged(variableName);

            if (variableName == "enableGhost")
            {

                if (enableGhost && ghost == null) ghost = GameEntity.CopyFrom(scene, physObject);
                if (ghost != null)
                {
                    SetupPhysLib(ghost);
                    ghost.SetGlobalFrame(physObject.GetGlobalFrame());
       
                    velocity = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));

                    if (enableGhost) velocity = Tuple.Create(new Vec3(0, 0, 0), new Vec3(0, 0, 0));  //reset object velocities states
                    else if (!enableGhost)
                    {
                        ghost.Remove(1);
                        ghost = null;
                    }
                }
            }       
        }

        protected override void OnEditorTick(float dt)
        {
            base.OnEditorTick(dt);
            if (ghost != null)
            {
                ghost.BodyFlag = BodyFlags.BodyOwnerEntity;
                if (enableGhost)
                {
                    TickExternalForces(ghost, dt);
                    TickFrame(ghost, dt);
                }
            }
        }

        protected override void OnInit()
        {
            base.OnInit();
            base.SetScriptComponentToTick(this.GetTickRequirement());

            physObject = base.GameEntity;
            scene = physObject.Scene;

            if (NavMeshPrefabName.Length > 0) AttachDynamicNavmeshToEntity();
            SetupPhysLib(physObject);
        }

        protected override void OnTick(float dt)
        {
            base.OnTick(dt);
        }

        protected override void OnPhysicsCollision(ref PhysicsContact contact)
        {
            base.OnPhysicsCollision(ref contact);
            SCEMath.DebugMessage("collision: " + contact.ToString());
        }

        private void TickExternalForces(GameEntity physObject, float dt)
        {
            MatrixFrame frame = physObject.GetGlobalFrame();
            float mass = physObject.Mass;
            Vec3 pos = physObject.GlobalPosition;
            Vec3 centerMass = physObject.CenterOfMass;

            Vec3 massPos = pos + frame.rotation.TransformToParent(centerMass);
            Vec3 massDir = frame.rotation.TransformToLocal(new Vec3(0, 0, -gravity * mass));
            //SCEMath.DebugMessage("mass: " + massDir.ToString());
            Tuple<Vec3, Vec3, Vec3, bool> weight = Tuple.Create(massDir, Vec3.Zero, massPos, false);

            Vec3 translationalVelocity = velocity.Item1;
            Tuple<Vec3, Vec3, Vec3, bool> translationalDrag = Tuple.Create(physLib.CalculateDragForces(translationalVelocity), Vec3.Zero, Vec3.Zero, true);

            Vec3 rotationalVelocity = velocity.Item2;
            Tuple<Vec3, Vec3, Vec3, bool> rotationalDrag = Tuple.Create(Vec3.Zero, physLib.CalculateDragForces(rotationalVelocity*10), Vec3.Zero, true);

            Vec3 testForce = frame.rotation.TransformToLocal(debugForce);

            Vec3 testMoment = debugMoment;
            Tuple<Vec3, Vec3, Vec3, bool> testRotation = Tuple.Create(testForce, testMoment, Vec3.Zero, true);

            List<Tuple<Vec3, Vec3, Vec3, bool>> loads = new List<Tuple<Vec3, Vec3, Vec3, bool>>();

            if (debugApplyTransDrag) loads.Add(translationalDrag);
            if (debugApplyRotDrag) loads.Add(rotationalDrag);

            if (applyDebugLoads) loads.Add(testRotation);
            if (!applyDebugLoads) loads.Add(weight);

            acceleration = physLib.ApplyLoadsAndGetAccelerations(physObject, loads);

        }

        private void TickFrame(GameEntity physObject, float dt)
        {
            MatrixFrame refFrame = physObject.GetGlobalFrame();
            MatrixFrame nextFrame = physObject.GetGlobalFrame();

            Vec3 translations = velocity.Item1;
            Vec3 prevRotations = velocity.Item2;
            Vec3 rotations = velocity.Item2;

            translations += acceleration.Item1 * dt;
            rotations += acceleration.Item2 * dt;
            
            rotations = SCEMath.LimitVectorComponents(rotations, SCEMath.GlobalMaxNum);
            nextFrame.rotation.RotateAboutSide(rotations.x * dt);
            nextFrame.rotation.RotateAboutForward(rotations.y * dt);
            nextFrame.rotation.RotateAboutUp(rotations.z * dt);

            if (debugAdjFrameVel) translations = physLib.AdjustFrameVelocityForRotation(translations, prevRotations, rotations, refFrame, nextFrame);

            translations = SCEMath.LimitVectorComponents(translations, SCEMath.GlobalMaxNum);
            nextFrame.Strafe(translations.x * dt);
            nextFrame.Advance(translations.y * dt);
            nextFrame.Elevate(translations.z * dt);

            velocity = Tuple.Create(translations, rotations);

            float groundHeight = scene.GetGroundHeightAtPosition(physObject.GlobalPosition, BodyFlags.BodyOwnerEntity);
            if (nextFrame.origin.z < groundHeight) nextFrame.origin.z = groundHeight;

            if (enableGhost && debugDisableVerticalTranslation) nextFrame.origin = this.physObject.GlobalPosition;

            physObject.SetGlobalFrame(nextFrame);
        }

        private void SetupPhysLib(GameEntity forPhysObject)
        {
            physLib = new SCEObjectPhysLib(forPhysObject, physObject.Mass, contactFriction, dragCoefficient, gravity);
        }

    }
}