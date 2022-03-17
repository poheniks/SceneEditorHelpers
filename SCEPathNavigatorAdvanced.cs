
using System;
using TaleWorlds.Engine;
using TaleWorlds.MountAndBlade;

namespace SceneEditorHelpers
{
    public enum PathNavigatorBehavior
    {
        Loop = 0,
        Stop = 1,
        Reverse = 2
    }

    public class SCE_PathNavigatorAdvanced : ScriptComponentBehavior
    {
        public string pathName;
        public float speed = 5f;
        public bool moveInEditor = true;
        public PathNavigatorBehavior behavior = PathNavigatorBehavior.Loop;

        private GameEntity entity;
        private Path path;
        private PathTracker pathTracker;
        private bool stopNavigator;
        private float reverse = 1f;

        protected override void OnEditorInit()
        {
            base.OnEditorInit();
            Init();
        }

        protected override void OnInit()
        {
            base.OnInit();
            base.SetScriptComponentToTick(TickRequirement.Tick);
            Init();
        }
        protected override void OnEditorTick(float dt)
        {
            base.OnEditorTick(dt);
            if (moveInEditor) MoveEntity(dt);
        }

        protected override void OnTick(float dt)
        {
            base.OnTick(dt);
            MoveEntity(dt);
        }
        protected override void OnEditorVariableChanged(string variableName)
        {
            base.OnEditorVariableChanged(variableName);
            Init();
            stopNavigator = false;
            if (CheckPathValidity())
            {
                pathTracker.Reset();
                entity.SetGlobalFrame(pathTracker.CurrentFrame);
            }
        }

        
        private void Init()
        {
            entity = base.GameEntity;

            Scene curScene = entity.Scene;
            if (curScene != null)
            {
                path = curScene.GetPathWithName(pathName);
                pathTracker = new PathTracker(path, entity.GetGlobalScale());
            }

        }
        private void MoveEntity(float dt)
        {
            if (!CheckPathValidity()) return;

            float threshold = 0.5f; //value in distance to loop/stop/reverse path entity before hitting the end; direction vector gets screwy/unreliable on the last path point matrix
            float endThreshold = pathTracker.GetPathLength() - threshold;

            if (pathTracker.TotalDistanceTraveled <= threshold && reverse < 0f) reverse *= -1f;
            
            if (pathTracker.TotalDistanceTraveled >= endThreshold)
            {
                switch (behavior)
                {
                    case PathNavigatorBehavior.Loop:
                        pathTracker.Reset();
                        break;
                    case PathNavigatorBehavior.Stop:
                        stopNavigator = true;
                        pathTracker.TotalDistanceTraveled = pathTracker.GetPathLength() - threshold;
                        entity.SetGlobalFrame(pathTracker.CurrentFrame);
                        break;
                    case PathNavigatorBehavior.Reverse:
                        reverse *= -1f;
                        break;
                    default:
                        pathTracker.Reset();
                        break;
                }

            }

            if (!stopNavigator)
            {
                speed = Math.Max(0f, speed);

                entity.SetGlobalFrame(pathTracker.CurrentFrame);
                pathTracker.Advance(speed*reverse*dt);
            }
        }

        private bool CheckPathValidity()
        {
            if (path != null && path.NumberOfPoints > 0 && pathTracker.GetPathLength() > 0) return true;
            return false;
        }
    }
}
