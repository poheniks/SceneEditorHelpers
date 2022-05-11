
using TaleWorlds.MountAndBlade;
using TaleWorlds.Engine;
using TaleWorlds.InputSystem;
using TaleWorlds.Core;
using TaleWorlds.Library;

namespace SceneEditorHelpers
{
    public class SceneEditorHelpersSubModule : MBSubModuleBase
    {
        public override void OnMissionBehaviorInitialize(Mission mission)
        {
            base.OnMissionBehaviorInitialize(mission);
            //if (mission.HasMissionBehaviour<TaleWorlds.MountAndBlade.Source.Missions.SimpleMountedPlayerMissionController>())
            if (mission.HasMissionBehavior<TaleWorlds.MountAndBlade.Source.Missions.SimpleMountedPlayerMissionController>() | mission.HasMissionBehavior<CustomBattleAgentLogic>())
            {
                mission.AddMissionBehavior(new SceneEditorTestMissionLogic());
                InformationManager.DisplayMessage(new InformationMessage("Scene Editor Helpers Loaded"));
                InformationManager.DisplayMessage(new InformationMessage("Press M to show help"));
            }
        }
    }

    public class SceneEditorTestMissionLogic : MissionLogic
    {
        bool init;
        Agent mainPlayer, mainHorse;
        GameEntity editorSpawn;
        GameEntity teleFlag;
        WorldPosition telePos;

        public override void OnMissionTick(float dt)
        {
            base.OnMissionTick(dt);
            if (Agent.Main != null && !init)
            {
                mainPlayer = Agent.Main;
                mainPlayer.SetInvulnerable(true);
                if (mainPlayer.HasMount)
                {
                    mainHorse = mainPlayer.MountAgent;
                    mainHorse.SetInvulnerable(true);
                }

                editorSpawn = Mission.Scene.GetFirstEntityWithScriptComponent<SCE_EditorSpawn>();
                if (editorSpawn != null) mainPlayer.TeleportToPosition(editorSpawn.GlobalPosition);
                init = true;
            }

            if (init && mainPlayer != null)
            {
                if (Input.IsKeyDown(InputKey.N))
                {
                    MBDebug.RenderDebugDirectionArrow(mainPlayer.GetChestGlobalPosition(), Vec3.Side, Colors.Red.ToUnsignedInteger());
                    MBDebug.RenderDebugDirectionArrow(mainPlayer.GetChestGlobalPosition(), Vec3.Forward, Colors.Green.ToUnsignedInteger());
                    MBDebug.RenderDebugDirectionArrow(mainPlayer.GetChestGlobalPosition(), Vec3.Up, Colors.Blue.ToUnsignedInteger());
                }

                if (Input.IsKeyPressed(InputKey.M))
                {
                    InformationManager.DisplayMessage(new InformationMessage("B: Return to editor spawn location"));
                    InformationManager.DisplayMessage(new InformationMessage("Q: Teleport to cursor"));
                    InformationManager.DisplayMessage(new InformationMessage("V: Teleport horse to player"));
                }

                if (Input.IsKeyPressed(InputKey.B))
                {
                    if (editorSpawn != null) mainPlayer.TeleportToPosition(editorSpawn.GlobalPosition);
                    else mainPlayer.TeleportToPosition(new Vec3(0, 0, 0));
                }
                if (Input.IsKeyDown(InputKey.Q))
                {
                    if (teleFlag == null)
                    {
                        teleFlag = GameEntity.Instantiate(Mission.Current.Scene, "bd_flag_1", mainPlayer.Frame);
                        teleFlag.BodyFlag = BodyFlags.DoNotCollideWithRaycast | BodyFlags.Disabled;
                    }
                    Ray rayCastEnd = new Ray(mainPlayer.GetEyeGlobalPosition(), mainPlayer.LookDirection, 500f);
                    float rayCastHitDist;

                    Mission.Scene.RayCastForClosestEntityOrTerrain(mainPlayer.GetEyeGlobalPosition() + new Vec3(0,0,0), rayCastEnd.EndPoint, out rayCastHitDist, rayThickness: 0.01f);
                    rayCastEnd.Reset(mainPlayer.GetEyeGlobalPosition(), mainPlayer.LookDirection, rayCastHitDist);

                    telePos = new WorldPosition(Mission.Current.Scene, rayCastEnd.EndPoint);
                    if (!telePos.IsValid) return;
                    teleFlag.SetLocalPosition(telePos.GetGroundVec3());
                }
                if (Input.IsKeyReleased(InputKey.Q))
                {
                    if (teleFlag != null) mainPlayer.TeleportToPosition(teleFlag.GlobalPosition);
                    teleFlag.Remove(0);
                    teleFlag = null;
                    if (mainHorse != null && mainHorse.AgentVisuals.GetSkeleton() != null) mainHorse.AgentVisuals.GetSkeleton().ResetCloths();
                }
                if (Input.IsKeyPressed(InputKey.V))
                {
                    if (mainHorse != null) mainHorse.TeleportToPosition(mainPlayer.Position);
                    if (mainHorse != null && mainHorse.AgentVisuals.GetSkeleton() != null) mainHorse.AgentVisuals.GetSkeleton().ResetCloths();
                }
            }
        }
    }

    public class SCE_EditorSpawn : ScriptComponentBehavior
    {
    }
}
