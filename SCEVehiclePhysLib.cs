using System;
using TaleWorlds.Library;
using TaleWorlds.Core;
using TaleWorlds.Engine;
using System.Collections.Generic;

namespace SceneEditorExtras
{
    //force = mass * acceleration
    //torque = moment of inertia * rotational accel
    //KE = 1/2 * mass * velocity^2
    //PE = mass * gravity * height
    //X = latitude
    //Y = longitude
    //Z = altitude
    
    class SCEVehiclePhysLib
    {
        private GameEntity vehicle;
        private float mass;
        private float power;
        private float topSpeed;
        private float turnRadius;
        private float grip;
        private float gravity;

        private Vec3 geometricCenter;
        private Vec3 geoToMassCenters;  //geometric center - mass center
        private Vec3 inertia;   //(pitch, roll, yaw) in terms of Bannerlord local coordinate system, about the vehicle's center of mass (as opposed to geometric center)

        public float rollCoeff;
        public float rollResistanceForce; //roll resistance force = rollCoeff * mass * gravity (NOTE: assume a fixed roll resistance force, independent of velocity, for simplicity)
        public float dragForce; //drag force = 1/2 * velocity^2 * dragCoeff
        public float dragCoeff;
        public float longitudinalAccel;
        public float lateralAcceleration;

        public SCEVehiclePhysLib(GameEntity v_vehicle, float v_mass, float v_power, float v_topSpeed, float v_turnRadius, float v_grip, float v_gravity)
        {
            vehicle = v_vehicle;
            mass = Math.Min(0.1f, v_mass);
            power = v_power;
            topSpeed = v_topSpeed;
            turnRadius = v_turnRadius;
            grip = v_grip;
            gravity = v_gravity;

            rollCoeff = 0.02f * grip;
            rollResistanceForce = rollCoeff * gravity;
            FindDragCoeff();
            FindCenters();
            FindInertiaAboutCoM();
        }

        public float CalculateLongitudinalAccel(float curVel, float analogPower)
        {

            GetDragForce(curVel);
            longitudinalAccel = ((power * analogPower) - dragForce - (curVel != 0 ? rollResistanceForce*(Math.Abs(curVel)/curVel) : 0f)) / mass;
            return longitudinalAccel;
        }

        public Tuple<Vec3, Vec3> CalculateWheelReaction(GameEntity wheel, float wheelRate, float wheelDisplacement)
        {
            if (wheel == null) return null;

            float wheelForce = wheelRate * wheelDisplacement;
            Vec3 forceVector = new Vec3(0, 0, wheelForce);
            Vec3 forcePosition = wheel.GetFrame().origin;

            return Tuple.Create(forceVector, forcePosition);

        }

        public float CalculateTurnRate(float curVel, float analogTurn)
        {
            if (analogTurn == 0f) return 0f;
            float targetRadius = turnRadius / analogTurn;

            float turnRate = Math.Abs(curVel) / targetRadius;
            float limitRate = (grip * gravity) / Math.Abs(curVel) * Math.Sign(analogTurn);

            if (turnRate < 0) turnRate = Math.Max(turnRate, limitRate);
            else if (turnRate > 0) turnRate = Math.Min(turnRate, limitRate);

            return turnRate;
        }

        public float CalculateWheelVerticalAccel()
        {
            return gravity*1;
        }

        private float GetDragForce(float curVel)
        {
            if (curVel == 0) return 0f;
            dragForce = 0.5f * signedPower(curVel, 2) * dragCoeff;
            return dragForce;
        }

        public Mat3 DampenRotationFrames(MatrixFrame prevFrame, MatrixFrame nextFrame, float dt)
        {
            Mat3 prevMat = prevFrame.rotation;
            Mat3 nextMat = nextFrame.rotation;
            
            Vec3 s = nextMat.s - (nextMat.s - prevMat.s) * dt * mass * 100;
            Vec3 f = nextMat.f - (nextMat.f - prevMat.f) * dt * mass * 100;
            Vec3 u = nextMat.u - (nextMat.u - prevMat.u) * dt * mass * 100;

            return new Mat3(s, f, u);
        }


        public Vec3 CalculateDragForces(Vec3 curVel)
        {

            Vec3 dragForceVector = new Vec3(0, 0, 0);
            dragForceVector.x = -GetDragForce(curVel.x);
            dragForceVector.y = -GetDragForce(curVel.y);
            dragForceVector.z = -GetDragForce(curVel.z);

            SCEMath.DebugMessage("dragForce: " + dragForceVector.ToString());
            SCEMath.DebugMessage("curVel: " + curVel.ToString());

            return dragForceVector;
        }


        public Vec3 AdjustFrameVelocityForRotation(Vec3 curVel, Vec3 prevRot, Vec3 curRot, MatrixFrame prevFrame, MatrixFrame nextFrame)
        {
            Vec3 newVel = curVel;

            Mat3 prevMatRot = prevFrame.rotation;
            Mat3 nextMatRot = nextFrame.rotation;
            Quaternion prevQuat = prevMatRot.ToQuaternion();
            Quaternion nextQuat = nextMatRot.ToQuaternion();

            Quaternion transQuat = prevQuat.TransformToLocal(nextQuat);
            Vec3 transVec = Quaternion.EulerAngleFromQuaternion(transQuat);

            //some error exists, presumably caused by sequentially rotating X, then Y, then Z
            newVel.RotateAboutX(-transVec.x);
            newVel.RotateAboutY(-transVec.y);  
            newVel.RotateAboutZ(-transVec.z);

            return newVel;
        }
        

        //accelerations refactor
        public Tuple<Vec3, Vec3> ApplyLoadsAndGetAccelerations(GameEntity vehicle, List<Tuple<Vec3, Vec3, Vec3, bool>> loads)    //force vector always local unless TransformToLocal;
        {
            Mat3 rotation = vehicle.GetGlobalFrame().rotation;
            Vec3 sumForce = new Vec3(0, 0, 0);
            Vec3 sumMoments = new Vec3(0, 0, 0);
            foreach (Tuple<Vec3, Vec3, Vec3, bool> load in loads) {
                bool locationIsLocal = load.Item4;
                Vec3 forceLocation = load.Item3;
                Vec3 momentsVector = load.Item2;
                Vec3 forceVector = load.Item1;
                
                Vec3 pos = vehicle.GlobalPosition + rotation.TransformToParent(geometricCenter);
                Vec3 momentArm;

                if (locationIsLocal) momentArm = (pos + forceLocation) - pos;
                else momentArm = pos - forceLocation;
                momentArm = rotation.TransformToLocal(momentArm);

                Vec3 moment = Vec3.CrossProduct(momentArm, forceVector);

                sumForce += forceVector;
                sumMoments += moment + momentsVector;
              
            }
            Vec3 translationalAccel = new Vec3(sumForce.x / mass, sumForce.y / mass, sumForce.z / mass);
            Vec3 rotationalAccel = new Vec3(sumMoments.x / inertia.x, sumMoments.y / inertia.y, sumMoments.z / inertia.z);
            //SCEMath.DebugMessage("b" + rotationalAccel.ToString());

            return Tuple.Create(translationalAccel, rotationalAccel);
        }

        //static properties
        private void FindDragCoeff()
        {
            dragCoeff = 2f * (power-rollResistanceForce) / (float)Math.Pow(topSpeed, 2);
        }
        private void FindCenters()
        {
            geometricCenter = SCEMath.AverageVectors(new List<Vec3>() { vehicle.GetBoundingBoxMax(), vehicle.GetBoundingBoxMin() });
            geoToMassCenters = geometricCenter - vehicle.CenterOfMass;
        }
        private void FindInertiaAboutCoM()
        {
            Vec3 max = vehicle.GetBoundingBoxMax();
            Vec3 min = vehicle.GetBoundingBoxMin();

            //inertias about geometric center
            float Ixx = (max.x - min.x) * (float)Math.Pow((max.z - min.z), 3) / 12; //pitch
            float Iyy = (max.z - min.z) * (float)Math.Pow((max.x - min.x), 3) / 12; //roll
            float Izz = (max.x - min.x) * (float)Math.Pow((max.y - min.y), 3) / 12; //yaw

            //parallel axis theorem
            float xxMd = mass * (float)Math.Pow(SCEMath.Resultant(geoToMassCenters.x, geoToMassCenters.z), 2);    
            float yyMd = mass * (float)Math.Pow(SCEMath.Resultant(geoToMassCenters.y, geoToMassCenters.z), 2);
            float zzMd = mass * (float)Math.Pow(SCEMath.Resultant(geoToMassCenters.x, geoToMassCenters.y), 2);

            inertia = new Vec3(Ixx + xxMd, Iyy + yyMd, Izz + zzMd)*0.1f; 

        }
        //misc 
        private float signedPower(float x, float y)
        {
            if (x == 0) return 0f;
            return (float)(Math.Pow(x, y)) * Math.Sign(x);
        }

        
    }
}
